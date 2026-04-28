#include "ecvt_controller.h"
#include "esp_log.h"
#include "esp_timer.h"

#include <macros.h>

static const char *TAG = "twai_sender";

ECVTController* ECVTController::instance = nullptr;
ECVTController::ECVTController(ShiftRegister* sr, bool wait_for_can)
    : primary_gts(ENGINE_GEARTOOTH_SENSOR_PIN, ENGINE_SAMPLE_WINDOW, ENGINE_COUNTS_PER_ROT), 
      secondary_gts(GEARBOX_GEARTOOTH_SENSOR_PIN, GEAR_SAMPLE_WINDOW, GEAR_COUNTS_PER_ROT),
      odrive(ECVT_ODRIVE_NODE_ID),
      shift_reg(sr), 
      control_cycle_count(0),
      last_engine_rpm_error(0),
      actuator_engage_position(0)
{
    instance = this; 
}

void ECVTController::init(bool wait_for_can) 
{
    if(!instance) {
        ESP_LOGE(TAG, "Instance not set");
        return;
    }

    /* Initialize limit switch pins */
    pinMode(ECVT_LIMIT_SWITCH_INBOUND_PIN, PinMode::INPUT_ONLY);
    pinMode(ECVT_LIMIT_SWITCH_OUTBOUND_PIN, PinMode::INPUT_ONLY);

    /* Initialize CAN BUS */
    odrive.set_ecvt_odrive();
    odrive.clear_errors();

    /* Wait for CAN Heartbeat - Blinking LEDs */
    if (wait_for_can) {
        vTaskDelay(pdMS_TO_TICKS(3000));
        bool state = true; 
        while (odrive.get_time_since_last_heartbeat() > 5e5) {
            shift_reg->write_all_leds(state);
            vTaskDelay(pdMS_TO_TICKS(100));
            state = !state;
        }
    }
    shift_reg->write_all_leds(true);

    bool homed = home_actuator(); 
    if (homed) {
        ESP_LOGI(TAG, "Actuator Homed!");
        shift_reg->write_all_leds(false);
    }

    /* Initialize Interrupts */
    attachInterrupt(ENGINE_GEARTOOTH_SENSOR_PIN, primary_isr, InterruptMode::RISING_EDGE);    
    attachInterrupt(GEARBOX_GEARTOOTH_SENSOR_PIN, secondary_isr, InterruptMode::RISING_EDGE);

    attachInterrupt(ECVT_LIMIT_SWITCH_INBOUND_PIN, inbound_isr, InterruptMode::RISING_EDGE);
    attachInterrupt(ECVT_LIMIT_SWITCH_OUTBOUND_PIN, outbound_isr, InterruptMode::RISING_EDGE);
    
    /* Begin Control Loop as Async Task */
    xTaskCreatePinnedToCore(taskWrapper, "ecvt_task", 4096, this, 10, &taskHandle, 1);

    const esp_timer_create_args_t timer_args = {
        .callback = &timerCallback,
        .arg = this,
        .name = "ecvt_timer"
    };

    esp_timer_create(&timer_args, &timerHandle);
    esp_timer_start_periodic(timerHandle, 10000);
}

/**
 * Run shift fork through homing sequence
 * 1. Shift out to outbound limit switch 
 * 2. Shift in to engaged limit switch 
*/
bool ECVTController::home_actuator(uint32_t timeout_ms) 
{
    /* Add delay to give ODrive time to initialize */
    vTaskDelay(pdMS_TO_TICKS(1000));
    odrive.set_axis_state(AXIS_STATE_CLOSED_LOOP_CONTROL); 
    odrive.set_controller_mode(CTRL_MODE_VELOCITY_CONTROL, INPUT_MODE_PASSTHROUGH);

    uint32_t start_time_ms = esp_timer_get_time() / 1e3;

    /* Shift out to outbound LS */
    start_time_ms = esp_timer_get_time() / 1e3;
    while((esp_timer_get_time() / 1e3 - start_time_ms) > timeout_ms) {
        odrive.set_input_vel(-ECVT_HOME_SPEED * ECVT_DIR);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    odrive.set_input_vel(0.0);
    odrive.set_absolute_position(-2.3f * ECVT_DIR);
    /* Shift in to engaged LS */
    odrive.set_input_pos(0.0f, 0, 0);
    start_time_ms = esp_timer_get_time() / 1e3;
    
    vTaskDelay(pdMS_TO_TICKS(1000));

    actuator_engage_position = 0.0f; 

    odrive.set_input_vel(0.0);

    return true; 
}

/**
 * Set actuator velocity based on PID controller.
 * Called through pinned task. 
*/
void ECVTController::control_loop()
{
    float dt_s = CONTROL_FUNCTION_INTERVAL_MS * SECONDS_PER_MS;
    float override = 0.0f;
    
    while(true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        odrive.request_total_charge_used();
        odrive.request_total_power_used();

        /* Grab sensor data */
        float primary_rpm = primary_gts.get_rpm();
        float gearbox_rpm = secondary_gts.get_rpm();
        float secondary_rpm = gearbox_rpm / GEAR_TO_SECONDARY_RATIO;
        
        /* Filter RPMs */
        float filtered_primary_rpm = engine_rpm_median_filter.update(primary_rpm);
        filtered_primary_rpm = engine_rpm_time_filter.update(filtered_primary_rpm);

        float filtered_secondary_rpm = gear_rpm_time_filter.update(gearbox_rpm);
        filtered_secondary_rpm = filtered_secondary_rpm / GEAR_TO_SECONDARY_RATIO;

        /* RPM error from target */
        float engine_rpm_error = filtered_primary_rpm - ECVT_TARGET_RPM;
        float filtered_engine_rpm_error = engine_rpm_derror_filter.update(engine_rpm_error);

        float engine_rpm_derror =
            (filtered_engine_rpm_error - last_engine_rpm_error) / dt_s;
        last_engine_rpm_error = filtered_engine_rpm_error;

        /* Calculate velocity command */
        float velocity_command =
            (engine_rpm_error * ACTUATOR_KP +
            MAX(0, engine_rpm_derror * ACTUATOR_KD));
        velocity_command = CLAMP(velocity_command, -VELOCITY_LIMIT, VELOCITY_LIMIT);
        
        odrive.set_axis_state(AXIS_STATE_CLOSED_LOOP_CONTROL);
        odrive.set_controller_mode(CTRL_MODE_VELOCITY_CONTROL, INPUT_MODE_PASSTHROUGH);

        /** Don't allow full shift out, keep shift fork at engaged limit switch when shifting out. */
        if (odrive.get_pos() * ECVT_DIR <= actuator_engage_position + 0.1 && velocity_command <= 0) {
            if(odrive.get_pos() * ECVT_DIR > actuator_engage_position - 0.1)
                odrive.set_axis_state(AXIS_STATE_IDLE);
            else
            {
                odrive.set_controller_mode(CTRL_MODE_POSITION_CONTROL, INPUT_MODE_PASSTHROUGH);
                odrive.set_input_pos(actuator_engage_position * ECVT_DIR, 0, 0);
            }
            shift_reg->write_led(3, true);

        }
        else{
            odrive.set_axis_state(AXIS_STATE_CLOSED_LOOP_CONTROL);
            odrive.set_controller_mode(CTRL_MODE_VELOCITY_CONTROL, INPUT_MODE_PASSTHROUGH);
            odrive.set_input_vel(velocity_command * ECVT_DIR, 0.0f);
        }

        /* Update telemetry buffer with current values */
        uint64_t time_us = esp_timer_get_time();
        Telemetry::back_buffer->time_ms = (float) time_us / 1e3;

        Telemetry::back_buffer->engine_rpm = primary_rpm;
        Telemetry::back_buffer->secondary_rpm = secondary_rpm; 

        Telemetry::back_buffer->filtered_engine_rpm = filtered_primary_rpm;
        Telemetry::back_buffer->filtered_secondary_rpm = filtered_secondary_rpm;

        Telemetry::back_buffer->target_rpm = ECVT_TARGET_RPM;
        Telemetry::back_buffer->engine_rpm_error = engine_rpm_error; 

        Telemetry::back_buffer->ecvt_velocity_command = velocity_command; 
        
        Telemetry::back_buffer->ecvt_velocity = odrive.get_vel() * ECVT_DIR;
        Telemetry::back_buffer->ecvt_pos = odrive.get_pos() * ECVT_DIR;
        Telemetry::back_buffer->ecvt_iq = odrive.get_iq();

        Telemetry::back_buffer->ecvt_bus_voltage = odrive.get_bus_voltage();
        Telemetry::back_buffer->ecvt_bus_current = odrive.get_bus_current();

        Telemetry::back_buffer->ecvt_total_charge_used = odrive.get_total_charge_used();
        Telemetry::back_buffer->ecvt_total_power_used = odrive.get_total_power_used();

        Telemetry::back_buffer->ecvt_inbound_limit_switch = get_inbound_limit(); 
        Telemetry::back_buffer->ecvt_outbound_limit_switch = get_outbound_limit(); 
        Telemetry::back_buffer->ecvt_engage_limit_switch = get_engage_limit(); 
        
        Telemetry::back_buffer = Telemetry::front_buffer.exchange(Telemetry::back_buffer);

        control_cycle_count++;
    }
}

bool ECVTController::get_outbound_limit() {
    return !digitalRead(ECVT_LIMIT_SWITCH_OUTBOUND_PIN);
}

bool ECVTController::get_inbound_limit() {
    return !digitalRead(ECVT_LIMIT_SWITCH_INBOUND_PIN);
}

bool ECVTController::get_engage_limit() {
    return !digitalRead(ECVT_LIMIT_SWITCH_ENGAGE_PIN);
}

void IRAM_ATTR ECVTController::primary_isr(void* p) {
    if (instance) {
        instance->primary_gts.update_isr(); 
    }
}

void IRAM_ATTR ECVTController::secondary_isr(void* p) {
    if (instance) {
        instance->secondary_gts.update_isr(); 
    }
}

void IRAM_ATTR ECVTController::outbound_isr(void* p) {
    if (instance) {
        instance->shift_reg->write_led(0, true);
        instance->odrive.set_input_vel(0.0f, 0.0f);
    }
}

void IRAM_ATTR ECVTController::inbound_isr(void* p) {
    if (instance) {
        instance->odrive.set_input_vel(0.0f, 0.0f);
    }
}

void ECVTController::timerCallback(void* arg)
{
    ECVTController* controller = (ECVTController*)arg;
    vTaskNotifyGiveFromISR(controller->taskHandle, NULL);
}

void ECVTController::taskWrapper(void* pvParameters)
{
    ((ECVTController*)pvParameters)->control_loop();
}
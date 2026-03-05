#include "ecvt_controller.h"
#include "esp_log.h"
#include "esp_timer.h"

#include <macros.h>

static const char *TAG = "twai_sender";

ECVTController::ECVTController(controller_mode_t mode_, ShiftRegister* sr, bool wait_for_can)
    : mode(mode_),
      primary_gts(ENGINE_GEARTOOTH_SENSOR_PIN, ENGINE_SAMPLE_WINDOW, ENGINE_COUNTS_PER_ROT), 
      secondary_gts(GEARBOX_GEARTOOTH_SENSOR_PIN, GEAR_SAMPLE_WINDOW, GEAR_COUNTS_PER_ROT),
      odrive(3),
      shift_reg(sr), 
      control_cycle_count(0)
{
    instance = this; 
}

void ECVTController::init(bool wait_for_can, Telemetry* telem) 
{
    this->telem = telem;
    pinMode(ECVT_LIMIT_SWITCH_INBOUND_PIN, PinMode::INPUT_ONLY);
    pinMode(ECVT_LIMIT_SWITCH_OUTBOUND_PIN, PinMode::INPUT_ONLY);

    odrive.init(CAN_TX_PIN, CAN_RX_PIN, CAN_BITRATE);
    odrive.start();
    odrive.clear_errors();

    shift_reg->write_all_leds(true);
    vTaskDelay(pdMS_TO_TICKS(2000));
    shift_reg->write_all_leds(false);

    attachInterrupt(ENGINE_GEARTOOTH_SENSOR_PIN, primary_isr, InterruptMode::RISING_EDGE);    
    attachInterrupt(GEARBOX_GEARTOOTH_SENSOR_PIN, secondary_isr, InterruptMode::RISING_EDGE);

    attachInterrupt(ECVT_LIMIT_SWITCH_INBOUND_PIN, inbound_isr, InterruptMode::RISING_EDGE);
    attachInterrupt(ECVT_LIMIT_SWITCH_OUTBOUND_PIN, outbound_isr, InterruptMode::RISING_EDGE);

    /* Wait for CAN Heartbeat - Blinking LEDs */
    if (wait_for_can) {
        vTaskDelay(pdMS_TO_TICKS(3000));
        ESP_LOGI(TAG, "time since heartbeat: %d", odrive.get_time_since_last_heartbeat()); 
        bool state = true; 
        while (odrive.get_time_since_last_heartbeat() > 5e5) {
            shift_reg->write_all_leds(state);
            vTaskDelay(pdMS_TO_TICKS(100));
            state = !state;
        }
        ESP_LOGI(TAG, "time since heartbeat: %d", odrive.get_time_since_last_heartbeat());
    }
    shift_reg->write_all_leds(false);

    bool homed = home_actuator(); 
    if (homed) {
        ESP_LOGI(TAG, "Actuator Homed!");
    }
    
    xTaskCreatePinnedToCore(taskWrapper, "ecvt_task", 4096, this, 10, &taskHandle, 1);

    const esp_timer_create_args_t timer_args = {
        .callback = &timerCallback,
        .arg = this,
        .name = "ecvt_timer"
    };
    esp_timer_create(&timer_args, &timerHandle);
    esp_timer_start_periodic(timerHandle, 10000);
}

void ECVTController::timerCallback(void* arg) {
    ECVTController* controller = (ECVTController*)arg;
    vTaskNotifyGiveFromISR(controller->taskHandle, NULL);
}
void ECVTController::taskWrapper(void* pvParameters) {
    ((ECVTController*)pvParameters)->control_loop();
}
void ECVTController::control_loop()
{
    ESP_LOGI(TAG, "Start");
    uint8_t node_id = 3;
    float dt_s = CONTROL_FUNCTION_INTERVAL_MS * SECONDS_PER_MS;
    float target_rpm = 3000;
    while(true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        float primary_rpm = primary_gts.get_rpm();
        float secondary_rpm = secondary_gts.get_rpm();
        
        float filtered_primary_rpm = engine_rpm_median_filter.update(primary_rpm);
        filtered_primary_rpm = engine_rpm_time_filter.update(filtered_primary_rpm);

        float filtered_secondary_rpm = gear_rpm_time_filter.update(secondary_rpm);
        filtered_secondary_rpm = filtered_secondary_rpm / GEAR_TO_SECONDARY_RATIO;

        float engine_rpm_error = filtered_primary_rpm - target_rpm;
        float filtered_engine_rpm_error = engine_rpm_derror_filter.update(engine_rpm_error);

        float engine_rpm_derror =
            (filtered_engine_rpm_error - last_engine_rpm_error) / dt_s;
        last_engine_rpm_error = filtered_engine_rpm_error;

        
        float velocity_command =
            (engine_rpm_error * ACTUATOR_KP +
            MAX(0, engine_rpm_derror * ACTUATOR_KD));
        velocity_command = CLAMP(velocity_command, -10.f, 10.f);
        odrive.set_axis_state(AXIS_STATE_CLOSED_LOOP_CONTROL);
        odrive.set_controller_mode(CTRL_MODE_VELOCITY_CONTROL, INPUT_MODE_VEL_RAMP);
        if(!(get_outbound_limit() && velocity_command > 0) && !(get_inbound_limit() && velocity_command < 0)) //Check signs on this
            odrive.set_input_vel(velocity_command, 0.0f);
        
        if(telem->lock())
        {
            uint64_t time_us = esp_timer_get_time();
            telem->data.time_ms = (float) time_us / 1e3;

            telem->data.engine_rpm = primary_rpm; 
            telem->data.secondary_rpm = secondary_rpm; 

            telem->data.filtered_engine_rpm = filtered_primary_rpm;
            telem->data.filtered_secondary_rpm = filtered_secondary_rpm;

            telem->data.target_rpm = target_rpm;
            telem->data.engine_rpm_error = engine_rpm_error; 

            telem->data.velocity_command = velocity_command; 
            
            telem->data.inbound_limit_switch = get_inbound_limit(); 
            telem->data.outbound_limit_switch = get_outbound_limit(); 
            telem->data.engage_limit_switch = get_engage_limit(); 
            telem->unlock();

        }
        control_cycle_count++;
    }
}

bool ECVTController::home_actuator(uint32_t timeout_ms) 
{
    /* Add delay to give ODrive time to initialize */
    vTaskDelay(pdMS_TO_TICKS(1000));
    odrive.set_axis_state(AXIS_STATE_CLOSED_LOOP_CONTROL); 
    odrive.set_controller_mode(CTRL_MODE_VELOCITY_CONTROL, INPUT_MODE_VEL_RAMP);

    /* Shift out to outbound LS */
    uint32_t start_time_ms = esp_timer_get_time() / 1e3;
    while(!get_outbound_limit()) {
        odrive.set_input_vel(-4.0);
        if ((esp_timer_get_time() - start_time_ms) > timeout_ms) {
            odrive.set_input_vel(0.0);
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    /* Shift in to inbound LS */
    start_time_ms = esp_timer_get_time() / 1e3;
    while(!get_engage_limit()) {
        odrive.set_input_vel(4.0);
        if ((esp_timer_get_time() - start_time_ms) > timeout_ms) {
            odrive.set_input_vel(0.0);
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    odrive.set_input_vel(0.0);

    return true; 
}

void ECVTController::control_function() {
    if (mode == NORMAL) {
        normal_control_function();
    } 
    else if (mode == BUTTON_SHIFT) {
        button_shift_control_function(); 
    }
} 

void ECVTController::button_shift_control_function() {
    /* Button Shift Control Function */
    bool button_1_pressed = digitalRead(BUTTON_1_PIN);
    bool button_2_pressed = digitalRead(BUTTON_2_PIN);
    bool button_3_pressed = digitalRead(BUTTON_3_PIN);
    bool button_4_pressed = digitalRead(BUTTON_4_PIN);

    float velocity = 10.0; 
    if (button_1_pressed) {
        odrive.set_axis_state(AXIS_STATE_IDLE); 
    } else if (button_2_pressed) {
        odrive.set_axis_state(AXIS_STATE_CLOSED_LOOP_CONTROL); 
    } else if (button_3_pressed) {
        odrive.set_input_vel(velocity); 
    } else if (button_4_pressed) {
        odrive.set_input_vel(-velocity);
    } else {
        odrive.set_input_vel(0.0); 
    }

    control_cycle_count++; 
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
        instance->odrive.set_input_vel(0.0f, 0.0f);
    }
}

void IRAM_ATTR ECVTController::inbound_isr(void* p) {
    if (instance) {
        instance->odrive.set_input_vel(0.0f, 0.0f);
    }
}
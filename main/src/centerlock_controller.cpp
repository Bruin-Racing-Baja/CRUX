#include <centerlock_controller.h>
#include <constants.h>
#include <odrive.h>
#include <macros.h>
#include <gpio_wrapper.h>
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "centerlock";

CenterlockController* CenterlockController::instance = nullptr;

CenterlockController::CenterlockController(gpio_num_t outbound_pin_, gpio_num_t inbound_pin_, gpio_num_t cl_led_) :
odrive(CENTERLOCK_ODRIVE_NODE_ID), 
curr_state(UNHOMED), 
outbound_pin(outbound_pin_), 
inbound_pin(inbound_pin_),
led(cl_led_),
led_state(LOW),
shift_start_time_ms(0)
{
    instance = this;
}

/* Initialize CenterlockController Objct, home and start controller */
void CenterlockController::init() 
{
    /* Initialize centerlock pins */
    pinMode(outbound_pin, PinMode::INPUT_PULLUP);
    pinMode(inbound_pin, PinMode::INPUT_PULLUP);

    pinMode(CENTERLOCK_LIMIT_SWITCH_INBOUND_PIN, PinMode::INPUT_PULLUP);
    pinMode(CENTERLOCK_LIMIT_SWITCH_OUTBOUND_PIN, PinMode::INPUT_PULLUP);
    pinMode(led, PinMode::OUTPUT_ONLY);

    pinMode(CENTERLOCK_SWITCH_1_PIN, PinMode::INPUT_PULLUP);
    pinMode(CENTERLOCK_SWITCH_2_PIN, PinMode::INPUT_PULLUP);

    odrive.set_centerlock_odrive();
    odrive.set_limits(CENTERLOCK_ODRIVE_VEL_LIMIT, CENTERLOCK_ODRIVE_CURRENT_LIMIT);

    /* Wait for CAN Heartbeat - Blinking LEDs */
    vTaskDelay(pdMS_TO_TICKS(3000));
    while (odrive.get_time_since_last_heartbeat() > 5e5) {
        digitalWrite(led, led_state);
        vTaskDelay(pdMS_TO_TICKS(100));
        led_state = !led_state;
    }
    led_state = HIGH; 
    digitalWrite(led, led_state);

    /* Home Centerlock Actuator */
    bool homed = home(); 
    if (homed) {
        ESP_LOGI(TAG, "Centerlock Homed!");
        led_state = LOW; 
        digitalWrite(led, led_state);
    }

    /* Attach limit switch interrupts */
    attachInterrupt(CENTERLOCK_SWITCH_1_PIN, shift_in_button_isr, InterruptMode::FALLING_EDGE);    
    attachInterrupt(CENTERLOCK_SWITCH_2_PIN, shift_out_button_isr, InterruptMode::FALLING_EDGE);
    
    /* Create and start Centerlock Controller task */
    xTaskCreatePinnedToCore(taskWrapper, "ecenterlock_task", 4096, this, 10, &taskHandle, 0);

    const esp_timer_create_args_t timer_args = {
        .callback = &timerCallback,
        .arg = this,
        .name = "ecvt_timer"
    };
    esp_timer_create(&timer_args, &timerHandle);
    esp_timer_start_periodic(timerHandle, 10000);
}

/* Shift to outbound on homing */ 
bool CenterlockController::home()
{
    vTaskDelay(pdMS_TO_TICKS(1000));

    odrive.set_axis_state(AXIS_STATE_CLOSED_LOOP_CONTROL);  
    odrive.set_controller_mode(CTRL_MODE_VELOCITY_CONTROL, INPUT_MODE_PASSTHROUGH);
    odrive.set_input_vel(-1 * CENTERLOCK_DIR * CENTERLOCK_HOME_VEL, 0.0f);
    ESP_LOGI(TAG, "Pre Homing");
    uint32_t timeout_ms = 20000;
    uint32_t start_time_ms = esp_timer_get_time() / 1e3;

    if (get_inbound_limit()) {
        odrive.set_axis_state(AXIS_STATE_IDLE); 
        curr_state = ENGAGED_4WD; 
        return true;
    }

    while(!get_outbound_limit()) {
        odrive.set_input_vel(-1 * CENTERLOCK_DIR * CENTERLOCK_HOME_VEL, 0.0f);
        if ((esp_timer_get_time() / 1e3 - start_time_ms) > timeout_ms) {
            odrive.set_axis_state(AXIS_STATE_IDLE);
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    odrive.set_axis_state(AXIS_STATE_IDLE);
    curr_state = DISENGAGED_2WD;

    ESP_LOGI(TAG, "HOMED");

    return true;
}   

/* Control loop for centerlock - State Machine */
void CenterlockController::control_loop() 
{
    int count = 10;
    int cycle_count = 0;
    while(true)
    {
        cycle_count++;
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* Sanity check for limit switches */
        if(get_outbound_limit() && get_inbound_limit()){
            curr_state = ERROR;
        }
        uint64_t cur_time_ms = esp_timer_get_time() / 1e3;
        uint64_t time_since_start_shift = shift_start_time_ms - cur_time_ms; 
        float velocity_command = 0.0f;
        switch(curr_state) {
            case UNHOMED:
                curr_state = ERROR; 
                break;

            case DISENGAGED_2WD:
                led_state = LOW; 
                digitalWrite(led, led_state);

                count--;
                if(count == 0){
                    count = 10;
                    odrive.set_axis_state(AXIS_STATE_IDLE);
                }
                break;

            case SHIFTING_TO_4WD:
                if(get_outbound_limit()) {
                    odrive.set_axis_state(AXIS_STATE_CLOSED_LOOP_CONTROL);
                    odrive.set_controller_mode(CTRL_MODE_VELOCITY_CONTROL, INPUT_MODE_PASSTHROUGH);
                    velocity_command = CENTERLOCK_VEL;
                    odrive.set_input_vel(CENTERLOCK_DIR * velocity_command);
                }

                if (get_inbound_limit()) {
                    odrive.set_axis_state(AXIS_STATE_IDLE);
                    curr_state = ENGAGED_4WD;
                }

                if (cycle_count % 10 == 0) {
                    led_state = !led_state; 
                    digitalWrite(led, led_state);
                }

                break;

            case ENGAGED_4WD:
                led_state = HIGH; 
                digitalWrite(led, led_state);
                break;

            case SHIFTING_TO_2WD:
                if(get_inbound_limit()) {
                    odrive.set_axis_state(AXIS_STATE_CLOSED_LOOP_CONTROL);
                    odrive.set_controller_mode(CTRL_MODE_VELOCITY_CONTROL, INPUT_MODE_PASSTHROUGH);
                    velocity_command = -1 * CENTERLOCK_VEL;
                    odrive.set_input_vel(CENTERLOCK_DIR * velocity_command);
                }
                    
                if (get_outbound_limit()) {
                    odrive.set_axis_state(AXIS_STATE_IDLE);
                    curr_state = DISENGAGED_2WD;
                }

                if (cycle_count % 10 == 0) {
                    led_state = !led_state; 
                    digitalWrite(led, led_state);
                }

                break;

            case ERROR:
                ESP_LOGI(TAG, "ERROR");
                odrive.set_axis_state(AXIS_STATE_IDLE);
        }

        if (cycle_count % 50 == 0) {
            ESP_LOGI(TAG, "State: %d, Inbound: %d, Outbound: %d, Velocity: %f", curr_state, get_inbound_limit(), get_outbound_limit(), velocity_command);
        }

        Telemetry::back_buffer->centerlock_velocity_command = velocity_command; 
        
        Telemetry::back_buffer->centerlock_velocity = odrive.get_vel() * CENTERLOCK_DIR;
        Telemetry::back_buffer->centerlock_pos = odrive.get_pos() * CENTERLOCK_DIR;
        Telemetry::back_buffer->centerlock_iq = odrive.get_iq();

        Telemetry::back_buffer->centerlock_bus_voltage = odrive.get_bus_voltage();
        Telemetry::back_buffer->centerlock_bus_current = odrive.get_bus_current();

        Telemetry::back_buffer->centerlock_inbound_limit_switch = get_inbound_limit(); 
        Telemetry::back_buffer->centerlock_outbound_limit_switch = get_outbound_limit(); 
    }
}

bool CenterlockController::get_outbound_limit() {
  return !digitalRead(CENTERLOCK_LIMIT_SWITCH_OUTBOUND_PIN);
}

bool CenterlockController::get_inbound_limit() {
  return !digitalRead(CENTERLOCK_LIMIT_SWITCH_INBOUND_PIN);
}

void IRAM_ATTR CenterlockController::shift_in_button_isr(void* p) {
    if (instance) {
        static uint64_t last_interrupt_time = 0;
        uint64_t interrupt_time = esp_timer_get_time() / 1e3;

        if (interrupt_time - last_interrupt_time > CENTERLOCK_BUTTON_DEBOUNCE_MS)
        {
            if (instance->get_state() == DISENGAGED_2WD || instance->get_state() == SHIFTING_TO_2WD)
            {
                instance->curr_state = SHIFTING_TO_4WD; 
                instance->shift_start_time_ms = esp_timer_get_time() / 1e3;
            }
        }
        last_interrupt_time = interrupt_time;
    }
}

void IRAM_ATTR CenterlockController::shift_out_button_isr(void* p) {
    if (instance) {
        static uint64_t last_interrupt_time = 0;
        uint64_t interrupt_time = esp_timer_get_time() / 1e3;
        if (interrupt_time - last_interrupt_time > CENTERLOCK_BUTTON_DEBOUNCE_MS)
        {
            if (instance->get_state() == ENGAGED_4WD || instance->get_state() == SHIFTING_TO_4WD)
            {
                instance->curr_state = SHIFTING_TO_2WD; 
                instance->shift_start_time_ms = esp_timer_get_time() / 1e3;
            }
        }
        last_interrupt_time = interrupt_time;
    }
}

void CenterlockController::taskWrapper(void* pvParameters) {
    ((CenterlockController*)pvParameters)->control_loop();
}

void CenterlockController::timerCallback(void* arg) {
    CenterlockController* controller = (CenterlockController*)arg;
    vTaskNotifyGiveFromISR(controller->taskHandle, NULL);
}
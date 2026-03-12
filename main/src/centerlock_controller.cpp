#include <centerlock_controller.h>
#include <constants.h>
#include <odrive.h>
#include <macros.h>
#include <gpio_wrapper.h>
#include <input_output/shift_register.h>
#include "esp_log.h"
#include "esp_timer.h"

uint32_t out_pin_ =  1;
uint32_t in_pin_ = 2;

static const char *TAG = "centerlock";

// make centerlock controller - has limit switch implemented 
CenterlockController* CenterlockController::instance = nullptr;
CenterlockController::CenterlockController(ShiftRegister* sr_, gpio_num_t outbound_pin_, gpio_num_t inbound_pin_) : 
odrive(3), 
shift_reg(sr_),
curr_state(UNHOMED), 
outbound_pin(outbound_pin_), 
inbound_pin(inbound_pin_),
shift_start_time_ms(0)
{
    instance = this;
}

void CenterlockController::init() 
{
    bool wait_for_can = true; /* Fix this */

    pinMode(outbound_pin, PinMode::INPUT_PULLUP);
    pinMode(inbound_pin, PinMode::INPUT_PULLUP);



    odrive.init(CAN_TX_PIN, CAN_RX_PIN, CAN_BITRATE);
    odrive.start();
    odrive.set_limits(ODRIVE_VEL_LIMIT, 1);

    shift_reg->write_all_leds(true);
    vTaskDelay(pdMS_TO_TICKS(2000));
    shift_reg->write_all_leds(false);

    /* Wait for CAN Heartbeat - Blinking LEDs */
    if (wait_for_can) {
        vTaskDelay(pdMS_TO_TICKS(3000));
        //ESP_LOGI(TAG, "time since heartbeat: %d", odrive.get_time_since_last_heartbeat()); 
        bool state = true; 
        while (odrive.get_time_since_last_heartbeat() > 5e5) {
            shift_reg->write_all_leds(state);
            vTaskDelay(pdMS_TO_TICKS(100));
            state = !state;
        }
        //ESP_LOGI(TAG, "time since heartbeat: %d", odrive.get_time_since_last_heartbeat());
    }
    shift_reg->write_all_leds(false);

    ESP_LOGI(TAG, "Start Homing");
    bool homed = home(); 
    if (homed) {
        ESP_LOGI(TAG, "Centerlock Homed!");
    }

    attachInterrupt(CENTERLOCK_SWITCH_1_PIN, shift_in_button_isr, InterruptMode::RISING_EDGE);    
    attachInterrupt(CENTERLOCK_SWITCH_2_PIN, shift_out_button_isr, InterruptMode::RISING_EDGE);
    
    xTaskCreatePinnedToCore(taskWrapper, "ecenterlock_task", 4096, this, 10, &taskHandle, 1);

    const esp_timer_create_args_t timer_args = {
        .callback = &timerCallback,
        .arg = this,
        .name = "ecvt_timer"
    };
    esp_timer_create(&timer_args, &timerHandle);
    esp_timer_start_periodic(timerHandle, 10000);
}

// Call homing sequence when first turned on -  if fully shifted in leave in 4 or shift out 
bool CenterlockController::home()
{
    vTaskDelay(pdMS_TO_TICKS(2000));

    // push all the way out 
    odrive.set_axis_state(AXIS_STATE_CLOSED_LOOP_CONTROL);  

    // set velocity on Odrive for the centerlock shifting
    odrive.set_input_vel(-1 * CENTERLOCK_DIR * CENTERLOCK_HOME_VEL, 0.0f);

    uint32_t timeout_ms = 5000;
    uint32_t start_time_ms = esp_timer_get_time() / 1e3;
    while(!get_outbound_limit()) {
        odrive.set_input_vel(-1 * CENTERLOCK_DIR * CENTERLOCK_HOME_VEL);
        if ((esp_timer_get_time() / 1e3 - start_time_ms) > timeout_ms) {
            odrive.set_input_vel(0.0, 0.0f);
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    odrive.set_input_vel(0.0, 0.0f);
    curr_state = DISENGAGED_2WD;

    return true;
}   

// Call centerlock controller
void CenterlockController::control_loop() 
{
    // initialize the switches / controller
    if(get_outbound_limit() && get_inbound_limit()){
        curr_state = ERROR;
    }

    uint64_t cur_time_ms = esp_timer_get_time() / 1e3;
    uint64_t time_since_start_shift = shift_start_time_ms - cur_time_ms; 

    switch(curr_state) {
        case UNHOMED:
            curr_state = ERROR; 

        case DISENGAGED_2WD:
            // if(req_4wd){
            //     curr_state = SHIFTING_TO_4WD;
            //     set_velocity(ECENTERLOCK_4WD_VEL);
            // }o
            break;

        case SHIFTING_TO_4WD:
            if(get_inbound_limit()){
                odrive.set_input_vel(CENTERLOCK_DIR * CENTERLOCK_VEL);
                curr_state = ENGAGED_4WD;
            }
            break;

        case ENGAGED_4WD:
            // if(req_2wd){
            //     curr_state = SHIFTING_TO_2WD;
            //     set_velocity(ECENTERLOCK_2WD_VEL);
            // }
            break;

        case SHIFTING_TO_2WD:
            if (time_since_start_shift > 2000) {
                
            }

            if(get_outbound_limit()){
                odrive.set_input_vel(-1 * CENTERLOCK_DIR * CENTERLOCK_VEL);
                curr_state = DISENGAGED_2WD;
            }
            break;

        case ERROR:
            odrive.set_input_vel(0, 0.0f);
    }
}

bool CenterlockController::get_outbound_limit() {
  return !digitalRead(CENTERLOCK_LIMIT_SWITCH_OUTBOUND_PIN);
}

bool CenterlockController::get_inbound_limit() {
  return !digitalRead(CENTERLOCK_LIMIT_SWITCH_OUTBOUND_PIN);
}

void CenterlockController::taskWrapper(void* pvParameters) {
    ((CenterlockController*)pvParameters)->control_loop();
}

void CenterlockController::timerCallback(void* arg) {
    CenterlockController* controller = (CenterlockController*)arg;
    vTaskNotifyGiveFromISR(controller->taskHandle, NULL);
}

void IRAM_ATTR CenterlockController::shift_in_button_isr(void* p) {
    if (instance) {
        if (instance->get_state() == DISENGAGED_2WD) {
            instance->curr_state = SHIFTING_TO_4WD; 
            instance->shift_start_time_ms = esp_timer_get_time() / 1e3;
        }
    }
}

void IRAM_ATTR CenterlockController::shift_out_button_isr(void* p) {
    if (instance) {
        if (instance->get_state() == ENGAGED_4WD) {
            instance->curr_state = SHIFTING_TO_2WD; 
            instance->shift_start_time_ms = esp_timer_get_time() / 1e3;
        }
    }
}


// void CenterlockController::set_velocity(float velocity) {
//   if (ODrive->get_axis_state() == AXIS_STATE_IDLE) {
//     ODrive->set_axis_state(AXIS_STATE_CLOSED_LOOP_CONTROL);
//   }

// // add if using more than just velocity controller
// //   if (odrive->set_controller_mode(ODrive::CONTROL_MODE_VELOCITY_CONTROL,
// //                                   ODrive::INPUT_MODE_VEL_RAMP) != 0) {
// //     return SET_VELOCITY_CAN_ERROR;
// //   }

//   velocity = CLAMP(velocity, -ODRIVE_VEL_LIMIT, ODRIVE_VEL_LIMIT);
//   if (ODrive->set_input_vel(velocity, 0) != 0) {
//     return SET_VELOCITY_CAN_ERROR;
//   }

//   return SET_VELOCITY_SUCCCESS;
// }
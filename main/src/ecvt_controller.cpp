#include "ecvt_controller.h"

#include <sensors/gear_tooth_sensor.h>
#include <input_output/shift_register.h>
#include <odrive.h> 
#include <constants.h>

#include "esp_timer.h"

ECVTController* ECVTController::instance = nullptr;

ECVTController::ECVTController(controller_mode_t mode_, ShiftRegister* sr, bool wait_for_can)
    : mode(mode_),
      primary_gts(ENGINE_GEARTOOTH_SENSOR_PIN, ENGINE_SAMPLE_WINDOW, ENGINE_COUNTS_PER_ROT), 
      secondary_gts(GEARBOX_GEARTOOTH_SENSOR_PIN, GEAR_SAMPLE_WINDOW, GEAR_COUNTS_PER_ROT),
      shift_reg(sr), 
      control_cycle_count(0)
{
    instance = this; 
    init(wait_for_can);
}

void ECVTController::init(bool wait_for_can) 
{
    /* Set Pin Modes */
    pinMode(ECVT_LIMIT_SWITCH_INBOUND_PIN, PinMode::INPUT_ONLY);
    pinMode(ECVT_LIMIT_SWITCH_OUTBOUND_PIN, PinMode::INPUT_ONLY);

    /* Initialize ODrive */
    odrive.init(CAN_TX_PIN, CAN_RX_PIN, CAN_BITRATE);
    odrive.start();
    odrive.clear_errors();

    /* Flash While Wait for CAN Heartbeat */
    if (wait_for_can) {
        uint32_t led_flash_time_ms = 100;
        while (odrive.get_time_since_last_heartbeat() > 1e6) {
            shift_reg->write_all_leds(esp_timer_get_time() % (led_flash_time_ms * 2) < led_flash_time_ms);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    shift_reg->write_all_leds(false);

    bool homed = home_actuator(); 
    if (homed) {
        /* Add log statement if homed */
    }

    /* Attach Interrupts */
    attachInterrupt(ENGINE_GEARTOOTH_SENSOR_PIN, primary_isr, InterruptMode::RISING_EDGE);    
    attachInterrupt(GEARBOX_GEARTOOTH_SENSOR_PIN, secondary_isr, InterruptMode::RISING_EDGE);

    attachInterrupt(ECVT_LIMIT_SWITCH_INBOUND_PIN, inbound_isr, InterruptMode::RISING_EDGE);
    attachInterrupt(ECVT_LIMIT_SWITCH_OUTBOUND_PIN, outbound_isr, InterruptMode::RISING_EDGE);
}

bool ECVTController::home_actuator() {
    /* Implement Actuator Homing */
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

void ECVTController::normal_control_function() {
    /* Normal Control Function */
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

void IRAM_ATTR ECVTController::primary_isr(void* p) {
    if (instance) {
        instance->primary_gts.update_isr(); 
    }
}

void IRAM_ATTR ECVTController::secondary_isr(void* p) {
    if (instance) {
        instance->primary_gts.update_isr(); 
    }
}

void IRAM_ATTR ECVTController::outbound_isr(void* p) {
    if (instance) {
        /* set velocity to 0, hard stop */
    }
}

void IRAM_ATTR ECVTController::inbound_isr(void* p) {
    if (instance) {
        /* Set velocity to 0, hard stop */
    }
}
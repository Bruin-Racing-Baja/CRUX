#ifndef CENTERLOCK_CONTROLLER_H
#define CENTERLOCK_CONTROLLER_H

#include <constants.h>
#include <odrive.h>
#include <input_output/shift_register.h>
#include "esp_timer.h"
#include "constants.h"

// test out git process

class CenterlockController {
public:
    enum State {
    UNHOMED,
    DISENGAGED_2WD,
    SHIFTING_TO_4WD,
    ENGAGED_4WD,
    SHIFTING_TO_2WD,
    ERROR
};
    static const uint32_t SET_TORQUE_SUCCESS = 0;
    static const uint32_t SET_TORQUE_OUT_LIMIT_SWITCH_ERROR = 1;
    static const uint32_t SET_TORQUE_CAN_ERROR = 2;

    static const uint32_t SET_VELOCITY_SUCCCESS = 0;
    static const uint32_t SET_VELOCITY_IN_LIMIT_SWITCH_ERROR = 1;
    static const uint32_t SET_VELOCITY_OUT_LIMIT_SWITCH_ERROR = 1;
    static const uint32_t SET_VELOCITY_CAN_ERROR = 2;

    static const uint32_t HOME_SUCCESS = 0;
    static const uint32_t HOME_CAN_ERROR = 1;
    static const uint32_t HOME_TIMEOUT_ERROR = 2;

    CenterlockController(ShiftRegister* sr_, gpio_num_t outbound_pin_, gpio_num_t inbound_pin_);
    void init();
    bool home(); 

    void control_loop(uint32_t timeout_ms);
    // void set_velocity(float velocity);
    // void fork_position(uint32_t timeout_ms, bool button_input_4WD, bool button_input_2WD);
    inline void set_state(State new_state) { curr_state = new_state; }
    inline State get_state(){return curr_state;}

    bool get_outbound_limit();
    bool get_inbound_limit(); 

    static IRAM_ATTR void shift_in_button_isr(void* p = nullptr);
    static IRAM_ATTR void shift_out_button_isr(void* p = nullptr);

private: 

    static CenterlockController* instance;

    ODrive odrive; 
    ShiftRegister* shift_reg; 
    
    State curr_state;
    
    gpio_num_t outbound_pin; 
    gpio_num_t inbound_pin; 

    TaskHandle_t taskHandle;
    esp_timer_handle_t timerHandle;

    uint64_t shift_start_time_ms; 

    void control_loop();
    static void timerCallback(void* arg);
    static void taskWrapper(void* pvParameters);
};

#endif // CENTERLOCK_CONTROLLER_H

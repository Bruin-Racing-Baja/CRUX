#ifndef CENTERLOCK_CONTROLLER_H
#define CENTERLOCK_CONTROLLER_H

#include <constants.h>
#include <odrive.h>
#include <input_output/shift_register.h>
#include "esp_timer.h"
#include "constants.h"
#include <telemetry.h>

class CenterlockController {
public:
    enum ButtonState {
        IN,
        OUT,
        UNKNOWN
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

    CenterlockController(gpio_num_t outbound_pin_, gpio_num_t inbound_pin_, gpio_num_t cl_led_);
    void init();
    bool home(); 

    void control_loop(uint32_t timeout_ms);

    bool get_outbound_limit();
    bool get_inbound_limit(); 

    static IRAM_ATTR void shift_in_button_isr(void* p = nullptr);
    static IRAM_ATTR void shift_out_button_isr(void* p = nullptr);

private: 
    void control_loop();
    static void timerCallback(void* arg);
    static void taskWrapper(void* pvParameters);

    static CenterlockController* instance;

    ODrive odrive;  

    ButtonState state;
    
    gpio_num_t outbound_pin; 
    gpio_num_t inbound_pin; 

    gpio_num_t led;
    bool led_state;

    TaskHandle_t taskHandle;
    esp_timer_handle_t timerHandle;

    uint64_t shift_start_time_ms; 
};

#endif // CENTERLOCK_CONTROLLER_H

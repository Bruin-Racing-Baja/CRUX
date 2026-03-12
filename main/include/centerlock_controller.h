#ifndef CENTERLOCK_CONTROLLER_H
#define CENTERLOCK_CONTROLLER_H

#include <constants.h>
#include <odrive.h?
#include "constants.h"

// test out git process

class CenterlockController {
public:
    enum State {
    UNHOMED,
    HOMING,
    ENGAGED_2WD,
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

    CenterlockController() {}
    centerlock_controller(ODrive *odrive);
    void control(uint32_t timeout_ms);
    void set_Velocity(float velocity);
    void fork_position(uint32_t timeout_ms, bool button_input_4WD, bool button_input_2WD);
    void set_State(State new_state) { curr_state = new_state; }
    State get_State(){return curr_state;}

    bool get_outbound_limit();
    


    static const float ECENTERLOCK_2WD_VEL = 3; 
    static const float ECENTERLOCK_4WD_VEL = 3;

private: 
    uint32_t Odrive_velocity; 
    State curr_state;
    centerlockLimitSwitch Centerlocklimitswitch;

};

#endif // CENTERLOCK_CONTROLLER_H

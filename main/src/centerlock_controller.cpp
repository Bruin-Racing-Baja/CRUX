#include <centerlock_controller.h>
#include <constants.h>
#include <odrive.h>
#include <macros.h>

uint32_t out_pin_ =  1;
uint32_t in_pin_ = 2;

// make centerlock controller - has limit switch implemented 

centerlock_controller::Centerlock_controller(ODrive *odrive) : odrive(odrive), curr_state(UNHOMED), centerlocklimitswitch(OUT_PIN, IN_PIN), num_tries(0), cycles_since_stopped(0) {}

// Call homing sequence when first turned on -  if fully shifted in leave in 4 or shift out 
void centerlock_controller::home(uint32_t timeout_ms){

    // push all the way out 
    if(ODrive->set_axis_state(ODrive::AXIS_STATE_CLOSED_LOOP_CONTROL)){
        return HOME_CAN_ERROR;
    }

    u32 start_time = millis();
    Serial.printf("disengaging...");   

    // set velocity on Odrive for the centerlock shifting
    set_velocity(ECENTERLOCK_HOME_VEL);
    float cur_pos = 0;

    return HOME_SUCCESS;
}   

// Call centerlock controller
void centerlock_controller::control(uint32_t timeout_ms, bool req_4wd, bool req_2wd) {

//     // protect the buttons 
//     if(req_4wd && req_2wd){
//     set_velocity(0);
//     return CONTROL_IDLE;
// }

    // initialize the switches / controller
    uint32_t now = millis();
    if(centerlocklimitswitch.is_inbound() && centerlocklimitswitch.is_outbound()){
    curr_state = ERROR;
    ODrive.set_velocity(0);
}
    switch(curr_state) {

    case UNHOMED:
        curr_state = HOMING;
        start_time = now;
        set_velocity(ECENTERLOCK_HOME_VEL);
        break;

    case HOMING:
        if(centerlocklimitswitch.is_outbound()){
            set_velocity(0);
            odrive->set_pos_rel(0);   // zero reference
            curr_state = ENGAGED_2WD;
        } 
        else if(now - start_time > timeout_ms){
            curr_state = ERROR;
        }
        break;

    case ENGAGED_2WD:
        if(req_4wd){
            curr_state = SHIFTING_TO_4WD;
            start_time = now;
            set_velocity(ECENTERLOCK_4WD_VEL);
        }
        break;

    case SHIFTING_TO_4WD:
        if(centerlocklimitswitch.is_inbound()){
            set_velocity(0);
            curr_state = ENGAGED_4WD;
        } 
        else if(now - start_time > timeout_ms){
            curr_state = ERROR;
        }
        break;

    case ENGAGED_4WD:
        if(req_2wd){
            curr_state = SHIFTING_TO_2WD;
            start_time = now;
            set_velocity(ECENTERLOCK_2WD_VEL);
        }
        break;

    case SHIFTING_TO_2WD:
        if(centerlocklimitswitch.is_outbound()){
            set_velocity(0);
            curr_state = ENGAGED_2WD;
        } 
        else if(now - start_time > timeout_ms){
            curr_state = ERROR;
            return CONTROL_TIMEOUT;
        }
        break;

    case ERROR:
        set_velocity(0);
        return CONTROL_ERROR;
    }
    return CONTROL_RUNNING;
}

void centerlock_controller::get_State(){
    return curr_state;
}


void centerlock_controller::set_velocity(float velocity) {
  if (ODrive->get_axis_state() == ODrive::AXIS_STATE_IDLE) {
    ODrive->set_axis_state(ODrive::AXIS_STATE_CLOSED_LOOP_CONTROL);
  }

// add if using more than just velocity controller
//   if (odrive->set_controller_mode(ODrive::CONTROL_MODE_VELOCITY_CONTROL,
//                                   ODrive::INPUT_MODE_VEL_RAMP) != 0) {
//     return SET_VELOCITY_CAN_ERROR;
//   }

  velocity = CLAMP(velocity, -ODRIVE_VEL_LIMIT, ODRIVE_VEL_LIMIT);
  if (ODrive->set_input_vel(velocity, 0) != 0) {
    return SET_VELOCITY_CAN_ERROR;
  }

  return SET_VELOCITY_SUCCCESS;
}
bool centerlock_controller::get_outbound_limit() {
  return !digitalRead(ECENTERLOCK_SENSOR_PIN);

}
#ifndef CENTERLOCK_LIMIT_SWITCH_H
#define CENTERLOCK_LIMIT_SWITCH_H

#include "gpio_wrapper.h"
#include "constants.h"
class CenterlockLimitSwitch {
public:
    CenterlockLimitSwitch(uint32_t out_pin_, uint32_t in_pin_) : out_pin(out_pin_), in_pin(in_pin_), engage_pressed(false), disengage_pressed(false) {
        pinMode(out_pin, PinMode::INPUT_ONLY);
        pinMode(in_pin, PinMode::INPUT_ONLY);
    }

    void update_isr_outbound(){
        int interrupt_pressed = digitalRead(out_pin);

        if (interrupt_pressed){
            disengage_pressed = true;
        } 
    }
    void update_isr_inbound(){
        int interrupt_pressed = digitalRead(in_pin);

        if (interrupt_pressed){
            engage_pressed = true;
        } 
    }
    
    bool is_outbound(){
        return disengage_pressed;
    }

    bool is_inbound(){
        return engage_pressed;
    }

    bool get_out_pin() {
       return out_pin;  
    }

    bool get_in_pin() {
        return in_pin; 
    }

private:
    uint32_t out_pin; 
    uint32_t in_pin;

    bool engage_pressed;
    bool disengage_pressed;
};

#endif // CENTERLOCK_LIMIT_SWITCH_H
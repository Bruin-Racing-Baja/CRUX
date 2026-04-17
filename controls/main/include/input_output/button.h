#ifndef BUTTON_H
#define BUTTON_H

#include "gpio_wrapper.h"
class Button {
public:
    Button(int pin) : pin(pin), button_pressed(false), last_state(false){
        pinMode(pin, PinMode::INPUT_PULLUP);
    }

    void button_isr(){
        if (!digitalRead(pin)) {
            button_pressed = true;
        } else {
            button_pressed = false;
        }
    }

    bool read_button_state(){
        return button_pressed;
    }
    bool read_last_state(){
        return last_state;
    }

private:
    int pin;
    bool button_pressed;
    bool last_state;
};

#endif // BUTTON_H
#ifndef BRAKE_POT_H
#define BRAKE_POT_H

#include "macros.h"
#include "gpio_wrapper.h"
#include <stdint.h>

class BrakePot {
public: 
    BrakePot(uint32_t pin_, uint32_t min_brake_, uint32_t max_brake_):
        pin(pin_),
        min_brake(min_brake_),
        max_brake(max_brake_)
        {
            pinMode(pin, PinMode::INPUT_ONLY);
        }

    inline uint32_t get_raw_brake() { return analogRead(pin); }
    inline float get_brake() { return map_int_to_float(analogRead(pin), min_brake, max_brake, 0, 1); }

private:
    const uint32_t pin; 
    const uint32_t min_brake; 
    const uint32_t max_brake; 
};

#endif // BRAKE_POT_H
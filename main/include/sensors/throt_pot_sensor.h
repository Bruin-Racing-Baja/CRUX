#ifndef THROT_POT_H
#define THROT_POT_H

#include "macros.h"
#include <stdint.h>

class ThrotPot {
public: 
    ThrotPot(uint32_t pin_, uint32_t min_throttle_, uint32_t max_throttle_):
        pin(pin_),
        min_throttle(min_throttle_),
        max_throttle(max_throttle_),
        raw_throttle(0),
        throttle(0.0f)
        {}
    
    inline uint32_t get_raw_throttle() { return analogRead(pin); }
    inline float get_throttle() { return map_int_to_float(analogRead(pin), min_throttle, max_throttle, 0, 1); }

private:
    const uint32_t pin; 

    const uint32_t min_throttle; 
    const uint32_t max_throttle; 

    uint32_t raw_throttle; 
    float throttle; 
};

#endif // THROT_POT_H
#ifndef SHOCK_POT_SENSOR_H
#define SHOCK_POT_SENSOR_H

#include <stdint.h>
#include "gpio_wrapper.h"

class ShockPotSensor {
    public:
        ShockPotSensor(uint32_t pin_, float max_travel_mm_ = 250.0f) : pin(pin_), max_travel_mm_(max_travel_mm_), raw(0), distance_mm(0.0f) {
            pinMode(pin, PinMode::INPUT_ONLY);
        }
        void update();

        inline uint32_t get_pin() const { return pin; }
        inline uint16_t get_raw() const { return raw; }
        float get_distance_mm() const { return distance_mm; }

    private:
        const uint32_t pin;
        const float max_travel_mm_;

        uint16_t raw;
        float distance_mm;

};

#endif // SHOCK_POT_SENSOR_H
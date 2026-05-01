#ifndef BRAKE_PRESSURE_SENSOR_H
#define BRAKE_PRESSURE_SENSOR_H

#include <stdint.h>
#include "gpio_wrapper.h"

class BrakePressureSensor {
    public:
        BrakePressureSensor(uint32_t pin_, float max_psi_) : pin(pin_), max_psi(max_psi_), raw(0), pressure_psi(0.0f) {
            pinMode(pin, PinMode::INPUT_ONLY);
        }
        void update();

        inline uint32_t get_pin() const { return pin; }
        inline uint16_t get_raw() const { return raw; }
        float get_pressure_psi() const { return pressure_psi; }

    private:
        const uint32_t pin;
        const float max_psi;

        uint16_t raw;
        float pressure_psi;

        static constexpr float R1 = 10000.0f; // 10k ohm resistor
        static constexpr float R2 = 30000.0f; // 30k ohm resistor
        static constexpr float ADC_MAX = 4095.0f; // 12-bit ADC max value
        static constexpr float V_REF = 3.3f; 
        static constexpr float V_MIN = 0.5f; 
        static constexpr float V_MAX = 4.5f;

};

#endif // BRAKE_PRESSURE_SENSOR_H
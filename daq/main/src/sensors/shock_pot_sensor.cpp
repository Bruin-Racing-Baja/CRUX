#include "sensors/shock_pot_sensor.h"

#define ADC_MAX 4095.0f

void ShockPotSensor::update() {
    raw = (uint16_t)analogRead(pin);
    distance_mm = (raw / ADC_MAX) * max_travel_mm_;
}
#include "sensors/shock_pot_sensor.h"

#define ADC_MAX 4095.0f // 12-bit ADC max value

void ShockPotSensor::update() {
    raw = (uint16_t)analogRead(pin); // Read raw ADC value (0–4095)
    distance_mm = (raw / ADC_MAX) * max_travel_mm_; 
    // TODO: Verify max and min raw values for calibration
}
#include "sensors/brake_pressure_sensor.h"

void BrakePressureSensor::update() {
    raw = (uint16_t)analogRead(pin); // Read raw ADC value (0–4095)

    float adc_voltage = (raw / ADC_MAX) * V_REF; // Convert raw ADC value to voltage
    float sensor_voltage = adc_voltage * (R1 + R2) / R2; // Calculate voltage across the sensor w voltage divider formula

    // clamp to valid range
    if (sensor_voltage < V_MIN) sensor_voltage = V_MIN; 
    if (sensor_voltage > V_MAX) sensor_voltage = V_MAX;

    pressure_psi = (sensor_voltage - V_MIN) * (max_psi / (V_MAX - V_MIN)); // Map voltage to pressure 
}
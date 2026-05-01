#include "sensors/brake_pressure_sensor.h"

#include "esp_log.h"

static const char *TAG = "sensor";

void BrakePressureSensor::update() {
    int raw_raw = analogRead(pin); // Read raw ADC value (0–4095)
    raw = (uint16_t)raw_raw; 
    ESP_LOGI(TAG, "raw_raw: %d", raw_raw);

    float adc_voltage = (raw / ADC_MAX) * V_REF; // Convert raw ADC value to voltage
    float sensor_voltage = adc_voltage * (R1 + R2) / R2; // Calculate voltage across the sensor w voltage divider formula

    // clamp to valid range
    // if (sensor_voltage < V_MIN) sensor_voltage = V_MIN; 
    // if (sensor_voltage > V_MAX) sensor_voltage = V_MAX;

    pressure_psi = (sensor_voltage - V_MIN) * (max_psi / (V_MAX - V_MIN)); // Map voltage to pressure 
}
#include "sensors/gear_tooth_sensor.h"
#include "esp_timer.h"
#include "constants.h"

/* Update sensor count and time values. */
void GearToothSensor::update_isr() {
    uint64_t cur_time_us = esp_timer_get_time();
    
    if (cur_time_us - last_time_us > min_time_diff_us) 
    {
        if (count % sample_window == 0) 
        {
            time_diff_us = cur_time_us - last_sample_time_us;
            last_sample_time_us = cur_time_us; 
        }
        count += 1; 
        last_time_us = cur_time_us;
    }
}

/* Calculate and return RPM from most recent gear count and time values. */
float GearToothSensor::get_rpm() {
    if (time_diff_us != 0) {
        rpm = (float)sample_window / counts_per_rot / 
                time_diff_us * US_PER_SECOND * SECONDS_PER_MINUTE; 
    }
    return rpm;
}
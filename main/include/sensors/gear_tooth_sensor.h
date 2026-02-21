#ifndef GEARTOOTH_SENSOR_H
#define GEARTOOTH_SENSOR_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "gpio_wrapper.h"

class GearToothSensor {
public: 
    GearToothSensor(uint32_t pin_, uint32_t sample_window_, uint32_t counts_per_rot_, uint32_t min_time_diff_us_ = 300): 
        pin(pin_), 
        sample_window(sample_window_),
        min_time_diff_us(min_time_diff_us_),
        counts_per_rot(counts_per_rot_),
        count(0),  
        last_sample_time_us(0),
        last_time_us(0),
        time_diff_us(0),
        rpm(0.0f)
        {
            pinMode(pin, PinMode::INPUT_ONLY);
        }

    void update_isr();

    /* Getter Functions */
    inline uint32_t get_pin() const {return pin; }
    inline uint32_t get_count() const { return count; }
    inline uint64_t get_time_diff_us() const { return time_diff_us; }
    float get_rpm();

private:
    /* Pin Number */
    const uint32_t pin; 

    /* Frequency (number of ticks) at which we collect timing information about GTS. */
    const uint32_t sample_window;

    /* Minimum allowable time between ticks - to prevent double counting */
    const uint32_t min_time_diff_us; 

    /* Number of teeth on the gear */
    const uint32_t counts_per_rot;

    /* Counter for each geartooth tick. */
    volatile uint32_t count;

    /* Timestamp of previously recorded gear tooth tick at sample window interval (us). */
    uint64_t last_sample_time_us;

    /* Timestamp of previous tick - for debouncing (us). */
    uint64_t last_time_us;

    /* Time between sample_window gear tooth ticks (us). */
    volatile uint64_t time_diff_us;

    /* RPM calculated from count and time_diff_us */
    float rpm;
};

#endif // GEARTOOTH_SENSOR_H
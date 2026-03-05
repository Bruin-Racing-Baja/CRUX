#ifndef CONSTANTS_H
#define CONSTANTS_H
#include <stdint.h>
#include <cstdint>
#include <macros.h>
/* Units */
constexpr float SECONDS_PER_MINUTE = 60.0; /* s / min */
constexpr float MS_PER_SECOND = 1.0e3;     /* ms / s */ 
constexpr float US_PER_SECOND = 1.0e6;     /* us / s */
constexpr float SECONDS_PER_MS = 1.0e-3;   /* s / ms */
constexpr float SECONDS_PER_US = 1.0e-6;   /* s / us */

constexpr float MM_PER_INCH = 25.4;              /* mm / inch */
constexpr float INCHES_PER_MM = 1 / MM_PER_INCH; /* inch / mm */

constexpr float FEET_PER_MILE = 5280.0; /* feet / mile */
constexpr float INCH_PER_FEET = 12.0;   /* inch / feet */

// Powertrain
constexpr uint32_t ENGINE_SAMPLE_WINDOW = 4;
constexpr uint32_t GEAR_SAMPLE_WINDOW = 10;

constexpr float ENGINE_COUNTS_PER_ROT = 16; // count / rot
constexpr float GEAR_COUNTS_PER_ROT = 6;    // count / rot

constexpr float GEAR_TO_WHEEL_RATIO = 58.0 / 19.0;                
constexpr float GEAR_TO_SECONDARY_RATIO = 17.0 / 50.0;

constexpr uint32_t CONTROL_FUNCTION_INTERVAL_MS = 10;

// CAN Values 
constexpr uint32_t CAN_BITRATE = 250000;

// Electronics Pins 
constexpr uint32_t ENGINE_GEARTOOTH_SENSOR_PIN = 17;
constexpr uint32_t GEARBOX_GEARTOOTH_SENSOR_PIN = 16;

constexpr gpio_num_t BRAKE_POT_PIN = GPIO_NUM_15;
constexpr gpio_num_t THROT_POT_PIN = GPIO_NUM_7;

constexpr uint32_t SR_SER_IN_PIN = 21;  // serin 
constexpr uint32_t SR_SHIFT_REG_CLK_PIN = 45; // srck
constexpr uint32_t SR_REG_CLK_PIN = 46; // rck

constexpr gpio_num_t ECVT_LIMIT_SWITCH_INBOUND_PIN = GPIO_NUM_12; 
constexpr gpio_num_t ECVT_LIMIT_SWITCH_OUTBOUND_PIN = GPIO_NUM_10; 
constexpr gpio_num_t ECVT_LIMIT_SWITCH_ENGAGE_PIN = GPIO_NUM_11; /* Not used */

constexpr uint32_t CENTERLOCK_LIMIT_SWITCH_INBOUND_PIN = 8;
constexpr uint32_t CENTERLOCK_LIMIT_SWITCH_OUTBOUND_PIN = 18;
constexpr gpio_num_t CENTERLOCK_LED_PIN = GPIO_NUM_2;
constexpr gpio_num_t CENTERLOCK_SWITCH_1_PIN = GPIO_NUM_13; 
constexpr gpio_num_t CENTERLOCK_SWITCH_2_PIN = GPIO_NUM_6; 
constexpr gpio_num_t CENTERLOCK_GTS_PIN = GPIO_NUM_9; 

constexpr gpio_num_t CAN_TX_PIN = GPIO_NUM_5;
constexpr gpio_num_t CAN_RX_PIN = GPIO_NUM_4; 

constexpr gpio_num_t BUTTON_4_PIN = GPIO_NUM_2; 
constexpr gpio_num_t BUTTON_3_PIN = GPIO_NUM_1; 
constexpr gpio_num_t BUTTON_2_PIN = GPIO_NUM_44; 
constexpr gpio_num_t BUTTON_1_PIN = GPIO_NUM_43; 

constexpr gpio_num_t EXTRA_IO_2_PIN = GPIO_NUM_45; 
constexpr gpio_num_t EXTRA_IO_1_PIN = GPIO_NUM_46; 
constexpr gpio_num_t EXTRA_GTS_PIN = GPIO_NUM_3; 

/* DAQ Pinouts */
constexpr int DAQ_LED_1_PIN = 21; 

constexpr int DAQ_BUTTON_A_PIN = 5;
constexpr int DAQ_BUTTON_B_PIN = 4;


constexpr uint32_t ENGINE_RPM_MEDIAN_FILTER_WINDOW = 3;

constexpr float ENGINE_RPM_ROTATION_FILTER_B[] = {
    0.8677114646, -3.305398989, 4.8804516238, -3.305398989, 0.8677114646};
constexpr float ENGINE_RPM_ROTATION_FILTER_A[] = {
    1.0, -3.5518051128, 4.8720546544, -3.0589928651, 0.7438198987};
constexpr size_t ENGINE_RPM_ROTATION_FILTER_M =
    COUNT_OF(ENGINE_RPM_ROTATION_FILTER_B);
constexpr size_t ENGINE_RPM_ROTATION_FILTER_N =
    COUNT_OF(ENGINE_RPM_ROTATION_FILTER_A);

constexpr float ENGINE_RPM_TIME_FILTER_B[] = {0.24523727525278557,
                                              0.24523727525278557};
constexpr float ENGINE_RPM_TIME_FILTER_A[] = {1.0, -0.5095254494944288};
constexpr size_t ENGINE_RPM_TIME_FILTER_M = COUNT_OF(ENGINE_RPM_TIME_FILTER_B);
constexpr size_t ENGINE_RPM_TIME_FILTER_N = COUNT_OF(ENGINE_RPM_TIME_FILTER_A);

constexpr float ENGINE_RPM_DERROR_FILTER_B[] = {0.07295965726826667,
                                                0.0729596572682667};
constexpr float ENGINE_RPM_DERROR_FILTER_A[] = {1.0, -0.8540806854634666};
constexpr size_t ENGINE_RPM_DERROR_FILTER_M =
    COUNT_OF(ENGINE_RPM_DERROR_FILTER_B);
constexpr size_t ENGINE_RPM_DERROR_FILTER_N =
    COUNT_OF(ENGINE_RPM_DERROR_FILTER_A);

constexpr float GEAR_RPM_TIME_FILTER_B[] = {
    0.007820208033497193, 0.015640416066994386, 0.007820208033497193};
constexpr float GEAR_RPM_TIME_FILTER_A[] = {1.0, -1.734725768809275,
                                            0.7660066009432638};
constexpr size_t GEAR_RPM_TIME_FILTER_M = COUNT_OF(GEAR_RPM_TIME_FILTER_B);
constexpr size_t GEAR_RPM_TIME_FILTER_N = COUNT_OF(GEAR_RPM_TIME_FILTER_A);

constexpr float THROTTLE_FILTER_B[] = {0.0591907, 0.0591907};
constexpr float THROTTLE_FILTER_A[] = {1., -0.88161859};
constexpr size_t THROTTLE_FILTER_M = COUNT_OF(THROTTLE_FILTER_B);
constexpr size_t THROTTLE_FILTER_N = COUNT_OF(THROTTLE_FILTER_A);

constexpr float ACTUATOR_KP = 0.03;   
constexpr float ACTUATOR_KI = 0.000;   
constexpr float ACTUATOR_KD = 0.000;

#endif // CONSTANTS_H
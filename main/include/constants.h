#ifndef CONSTANTS_H
#define CONSTANTS_H
#include <stdint.h>

#include <cstdint>

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

// Electronics Pins 
constexpr uint32_t ENGINE_GEARTOOTH_SENSOR_PIN = 17;
constexpr uint32_t GEARBOX_GEARTOOTH_SENSOR_PIN = 16;

constexpr uint32_t SR_SER_IN_PIN = 21;  // serin 
constexpr uint32_t SR_SHIFT_REG_CLK_PIN = 45; // srck
constexpr uint32_t SR_REG_CLK_PIN = 46; // rck

constexpr int CENTERLOCK_LIMIT_SWITCH_INBOUND_PIN = 8;
constexpr int CENTERLOCK_LIMIT_SWITCH_OUTBOUND_PIN = 18;

constexpr int CONTROLS_BUTTON_4_PIN = 2;

/* DAQ Pinouts */
constexpr int DAQ_LED_1_PIN = 21; 

constexpr int DAQ_BUTTON_A_PIN = 5;
constexpr int DAQ_BUTTON_B_PIN = 4;

#endif // CONSTANTS_H
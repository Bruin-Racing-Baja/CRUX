#ifndef DAQ_CONSTANTS_H
#define DAQ_CONSTANTS_H

constexpr int SHOCK_RL_PIN = 1;
constexpr int SHOCK_RR_PIN = 8;
constexpr int BRAKE_PRESSURE_FRONT_SENSOR_PIN = 18;
constexpr int BRAKE_PRESSURE_BACK_SENSOR_PIN = 19;

constexpr int GPS_TX_PIN = 43; // 22
constexpr int GPS_RX_PIN = 44; // 23

#define IMU_MOSI_PIN  42 //22 DI
#define IMU_MISO_PIN  9 //25 SDA
#define IMU_SCK_PIN   10 //26 SCL
#define IMU_CS_PIN    41 //23 CS
#define IMU_INT_PIN   39 //33 INT
#define IMU_RST_PIN   47 //21 RST

#endif // DAQ_CONSTANTS_H
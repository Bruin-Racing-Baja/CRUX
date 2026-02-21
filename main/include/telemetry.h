#ifndef TELEMETRY_H
#define TELEMETRY_H
#include <cstdint>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_rom_crc.h"
#include <cstring>
#include "driver/uart.h"
#include "driver/gpio.h"

#pragma pack(push, 1)
struct VehicleData {
    float timestamp;
    float steer_angle;
    float throttle_pos;
    float brake_pressure;
    float wheel_speed_fl;
    float wheel_speed_fr;
    float wheel_speed_rl;
    float wheel_speed_rr;
    float battery_voltage;
    float current_draw;
    
};
#pragma pack(pop)

#pragma pack(push, 1)
struct TelemetryPacket {
    // Header
    uint8_t start_byte_1;
    uint8_t start_byte_2;
    uint16_t packet_type;
    uint32_t sequence_number;
    VehicleData payload; 
    uint32_t checksum;
};
#pragma pack(pop)

class Telemetry {
public:
    static void init();
    static void send_data();
    static VehicleData data;
    static bool lock(TickType_t timeout_ticks = portMAX_DELAY);
    static void unlock();
    
private:
    static int sequence_number;
    static SemaphoreHandle_t data_mutex;
    
};

#endif // TELEMETRY_H
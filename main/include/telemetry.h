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
#include <atomic>

#pragma pack(push, 1)
struct VehicleData {
    float time_ms;
    float engine_count; 
    float gear_count; 

    float engine_rpm; 
    float secondary_rpm; 

    float filtered_engine_rpm; 
    float filtered_secondary_rpm; 

    float target_rpm; 
    float engine_rpm_error; 

    float velocity_command; 
    
    float inbound_limit_switch;
    float outbound_limit_switch; 
    float engage_limit_switch;
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
    static void send_data(void* pvParameters = nullptr);
    static VehicleData buffer_a, buffer_b;
    static VehicleData* back_buffer;
    static std::atomic<VehicleData*> front_buffer;
    static bool lock(TickType_t timeout_ticks = portMAX_DELAY);
    static void unlock();
    
private:
    static int sequence_number;
    static SemaphoreHandle_t data_mutex;
    
};

#endif // TELEMETRY_H
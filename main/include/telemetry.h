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

/* Values to telemeter */
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

    float ecvt_velocity_command; 

    float ecvt_velocity;
    float ecvt_pos;
    float ecvt_iq;

    float ecvt_bus_voltage;
    float ecvt_bus_current;

    float ecvt_total_charge_used;
    float ecvt_total_power_used;
    
    float ecvt_inbound_limit_switch;
    float ecvt_outbound_limit_switch; 
    float ecvt_engage_limit_switch;

    float centerlock_velocity_command; 

    float centerlock_velocity;
    float centerlock_pos;
    float centerlock_iq;

    float centerlock_bus_voltage;
    float centerlock_bus_current;

    float centerlock_total_charge_used;
    float centerlock_total_power_used;
    
    float centerlock_inbound_limit_switch;
    float centerlock_outbound_limit_switch; 
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

class Telemetry
{
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
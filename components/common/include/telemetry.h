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
struct ControlsData {
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
struct DaqData {
    float time_ms;
    float shock_rl_mm;
    float shock_rr_mm;
    float shock_rl_raw;
    float shock_rr_raw;

    //gps
    float longitude;
    float latitude;
    float mps;
    float heading_deg;

    //brake 
    float brake_pressure_front_psi;
    float brake_pressure_front_raw;
    float brake_pressure_back_psi;
    float brake_pressure_back_raw;

    //imu
    float ax;          
    float ay;
    float az;
    float qw;
    float qi;
    float qj;
    float qk;
    
};
#pragma pack(pop)

#pragma pack(push, 1)
template<typename T>
struct TelemetryPacket {
    // Header
    uint8_t start_byte_1;
    uint8_t start_byte_2;
    uint16_t packet_type;
    uint32_t sequence_number;
    T payload; 
    uint32_t checksum;
};
#pragma pack(pop)

template<typename T, uint16_t PACKET_TYPE>
class TelemetryBase
{
public:
    static void init();
    static void send_data(void* pvParameters = nullptr);
    static T buffer_a, buffer_b;
    static T* back_buffer;
    static std::atomic<T*> front_buffer;
    static bool lock(TickType_t timeout_ticks = portMAX_DELAY);
    static void unlock();
    
private:
    static int sequence_number;
    static SemaphoreHandle_t data_mutex;
    
};

using ControlsTelemetry = TelemetryBase<ControlsData, 0x01>;
using DaqTelemetry = TelemetryBase<DaqData, 0x02>;

// Alias to match controls name conventions, can be changed
using VehicleData = ControlsData;
using Telemetry = ControlsTelemetry;

#endif // TELEMETRY_H
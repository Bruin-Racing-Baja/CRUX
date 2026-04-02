#include "telemetry.h"
#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
static const char* TAG = "ODrive";

VehicleData Telemetry::buffer_a = {0};
VehicleData Telemetry::buffer_b = {0};
VehicleData* Telemetry::back_buffer = &Telemetry::buffer_a;
std::atomic<VehicleData*> Telemetry::front_buffer{&Telemetry::buffer_b};
SemaphoreHandle_t Telemetry::data_mutex;

int Telemetry::sequence_number = 0;
void Telemetry::init()
{
    
    data_mutex = xSemaphoreCreateMutex();
    sequence_number = 0;
    usb_serial_jtag_driver_config_t usb_config = {
        .tx_buffer_size = 1024,
        .rx_buffer_size = 1024,
    };
    
    // Install the driver
    usb_serial_jtag_driver_install(&usb_config);

}

bool Telemetry::lock(TickType_t timeout_ticks) {
    return xSemaphoreTake(data_mutex, timeout_ticks) == pdTRUE;
}

void Telemetry::unlock() {
    xSemaphoreGive(data_mutex);
} 

void Telemetry::send_data(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100ms period
    
    for(;;){
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        if(Telemetry::front_buffer.load() == nullptr) {
            continue;
        }
        TelemetryPacket packet;
        packet.start_byte_1 = 0xFA;
        packet.start_byte_2 = 0xCE;
        packet.packet_type = 0x01; 
        packet.sequence_number = Telemetry::sequence_number++;
        
        //lock();
        memcpy(&packet.payload, Telemetry::front_buffer.load(), sizeof(VehicleData));
       // unlock();

        packet.checksum = esp_rom_crc32_le(0, (uint8_t*)&packet, sizeof(TelemetryPacket) - sizeof(packet.checksum));

        usb_serial_jtag_write_bytes(&packet, sizeof(TelemetryPacket), pdMS_TO_TICKS(100));
    }
}
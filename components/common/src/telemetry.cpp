#include "telemetry.h"
#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
static const char* TAG = "Telem";

template<typename T, uint16_t PACKET_TYPE>
T TelemetryBase<T, PACKET_TYPE>::buffer_a = {0};

template<typename T, uint16_t PACKET_TYPE>
T TelemetryBase<T, PACKET_TYPE>::buffer_b = {0};

template<typename T, uint16_t PACKET_TYPE>
T* TelemetryBase<T, PACKET_TYPE>::back_buffer = &TelemetryBase<T, PACKET_TYPE>::buffer_a;

template<typename T, uint16_t PACKET_TYPE>
std::atomic<T*> TelemetryBase<T, PACKET_TYPE>::front_buffer{&TelemetryBase<T, PACKET_TYPE>::buffer_b};

template<typename T, uint16_t PACKET_TYPE>
SemaphoreHandle_t TelemetryBase<T, PACKET_TYPE>::data_mutex = nullptr;

template<typename T, uint16_t PACKET_TYPE>
int TelemetryBase<T, PACKET_TYPE>::sequence_number = 0;

template<typename T, uint16_t PACKET_TYPE>
void TelemetryBase<T, PACKET_TYPE>::init()
{ 
    data_mutex = xSemaphoreCreateMutex();
    sequence_number = 0;
    usb_serial_jtag_driver_config_t usb_config = {
        .tx_buffer_size = 1024,
        .rx_buffer_size = 1024,
    };
    
    /* Install the driver */
    usb_serial_jtag_driver_install(&usb_config);
}

template<typename T, uint16_t PACKET_TYPE>
bool TelemetryBase<T, PACKET_TYPE>::lock(TickType_t timeout_ticks) {
    return xSemaphoreTake(data_mutex, timeout_ticks) == pdTRUE;
}

template<typename T, uint16_t PACKET_TYPE>
void TelemetryBase<T, PACKET_TYPE>::unlock() {
    xSemaphoreGive(data_mutex);
} 

template<typename T, uint16_t PACKET_TYPE>
void TelemetryBase<T, PACKET_TYPE>::send_data(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100ms period
    
    for(;;)
    { 
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        if(front_buffer.load() == nullptr) {
            continue;
        }
        TelemetryPacket<T> packet;
        packet.start_byte_1 = 0xFA;
        packet.start_byte_2 = 0xCE;
        packet.packet_type = PACKET_TYPE; 
        packet.sequence_number = sequence_number++;
        
        memcpy(&packet.payload, front_buffer.load(), sizeof(T));

        packet.checksum = esp_rom_crc32_le(0, (uint8_t*)&packet, sizeof(TelemetryPacket<T>) - sizeof(packet.checksum));

        int written = usb_serial_jtag_write_bytes(&packet, sizeof(TelemetryPacket<T>), pdMS_TO_TICKS(100));
        if (written < 0) {
            ESP_LOGE(TAG, "Failed to write telem packet");
        }
    }
}

template class TelemetryBase<ControlsData, 0x01>;
template class TelemetryBase<DaqData,      0x02>;
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "constants.h"
#include "gpio_wrapper.h"
#include "telemetry.h"
#include "sensors/imu_sensor.h"
#include "sensors/shock_pot_sensor.h"
#include "sensors/gps_sensor.h"

static const char *TAG = "daq_main";

// rear shock pots
ShockPotSensor shock_rear_left(SHOCK_RL_PIN);
ShockPotSensor shock_rear_right(SHOCK_RR_PIN);

// gps sensor
GPS gps(GPS_TX_PIN, GPS_RX_PIN);

//imu sensor
IMUSensor imu(IMU_MOSI_PIN, IMU_MISO_PIN, IMU_SCK_PIN,
              IMU_CS_PIN, IMU_INT_PIN, IMU_RST_PIN);

// DAQ task and timer handles
static TaskHandle_t daq_task_handle = nullptr;
static esp_timer_handle_t daq_timer_handle = nullptr;

// ISR for DAQ timer 
static void daq_timer_callback(void* arg) {
    if (daq_task_handle != nullptr) {
        vTaskNotifyGiveFromISR(daq_task_handle, NULL);
    }
}

// DAQ task: waits for timer notification, reads shock sensor data, updates telemetry buffers
static void daq_task(void* pvParameters) {
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        shock_rear_left.update();
        shock_rear_right.update();

        gps.update();
        imu.update();
        // vTaskDelay(pdMS_TO_TICKS(1000)); // 1 second
        // printf("Latitude: %f, Longitude: %f, Speed: %f, Has fix: %f\n", 
        //     gps.get_latitude(), gps.get_longitude(), gps.get_speed_mps(), gps.get_has_fix());

        // timestamp and telemetry buffer update
        uint64_t time_us = esp_timer_get_time();
        DaqTelemetry::back_buffer->time_ms = (float)time_us / 1e3;
        
        DaqTelemetry::back_buffer->shock_rl_mm = shock_rear_left.get_distance_mm();
        DaqTelemetry::back_buffer->shock_rr_mm = shock_rear_right.get_distance_mm();
        DaqTelemetry::back_buffer->shock_rl_raw = shock_rear_left.get_raw();
        DaqTelemetry::back_buffer->shock_rr_raw = shock_rear_right.get_raw();
        
        DaqTelemetry::back_buffer->latitude = gps.get_latitude();
        DaqTelemetry::back_buffer->longitude = gps.get_longitude();
        DaqTelemetry::back_buffer->mps = gps.get_speed_mps();
        DaqTelemetry::back_buffer->heading_deg = gps.get_heading_deg();

        DaqTelemetry::back_buffer->yaw_deg   = imu.yaw();
        DaqTelemetry::back_buffer->pitch_deg = imu.pitch();
        DaqTelemetry::back_buffer->roll_deg  = imu.roll();
        DaqTelemetry::back_buffer->ax        = imu.ax();
        DaqTelemetry::back_buffer->ay        = imu.ay();
        DaqTelemetry::back_buffer->az        = imu.az();
        DaqTelemetry::back_buffer->gx        = imu.gx();
        DaqTelemetry::back_buffer->gy        = imu.gy();
        DaqTelemetry::back_buffer->gz        = imu.gz();
        // static int counter = 0;
        // if (++counter % 20 == 0) {  
        //     ESP_LOGI(TAG, "RL: %.2f mm | RR: %.2f mm",
        //             shock_rear_left.get_distance_mm(),
        //             shock_rear_right.get_distance_mm());
        // }
        DaqTelemetry::back_buffer = DaqTelemetry::front_buffer.exchange(DaqTelemetry::back_buffer);
    }
}

extern "C" void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(500));

    DaqTelemetry::init();

    vTaskDelay(pdMS_TO_TICKS(500));

    imu.init();
    printf("IMU init done, has_fix=%d\n", imu.is_ready());

    // Telem task and DAQ task creation
    xTaskCreatePinnedToCore(DaqTelemetry::send_data, "telemetry_task", 4096,  nullptr, tskIDLE_PRIORITY + 5, NULL, 0);
    xTaskCreatePinnedToCore(daq_task, "daq_task", 4096, nullptr, 10, &daq_task_handle, 0);

    // DAQ timer setup
    const esp_timer_create_args_t timer_args = {
        .callback = &daq_timer_callback,
        .arg = nullptr,
        .name = "daq_timer"
    };

    // update every 10 ms (100 Hz)
    esp_timer_create(&timer_args, &daq_timer_handle);
    esp_timer_start_periodic(daq_timer_handle, 10000); 

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

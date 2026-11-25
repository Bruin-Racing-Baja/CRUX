
#include "sd_logger.h"
#include "sensors/sensor.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>

static const char* TAG = "MINI_DAQ";

#include "constants.h"

ShockPotSensor shock_left(SHOCK_RL_PIN);
ShockPotSensor shock_right(SHOCK_RR_PIN);
IMUSensor imu;

extern "C" void app_main(void) {
    SDLogger logger;
    imu.init();

    if (!logger.init()) { //Initialize SD card
        ESP_LOGE(TAG, "SD logger initialization failed");
        return;
    }

    ESP_LOGI(TAG, "Starting data logging"); //Start logging

    while (true) {
        // Read the sensors
        shock_left.update();
        shock_right.update();
        imu.update();

        // Get the current time
        long long time_ms = esp_timer_get_time() / 1000;

        // Format the data into one CSV row
        char row[200];

        snprintf(row, sizeof(row), //Add info from shock sensors/ imu to list
                "%lld,%.2f,%.2f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f",
                time_ms,
                shock_left.get_distance_mm(),
                shock_right.get_distance_mm(),
                imu.ax(), imu.ay(), imu.az(),
                imu.qw(), imu.qi(), imu.qj(), imu.qk());

        // Write the row to the SD card
        if (!logger.write(row)) { //add list to sd card
            ESP_LOGE(TAG, "Logging failed");
            logger.close();
            return;
        }

        // Wait about 10 ms (target: 100 Hz)
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


#include "sd_logger.h"
#include "sd_config.h"

#include "sensors/sensor.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_vfs_fat.h"
#include "driver/spi_common.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"

static const char* TAG = "SD_LOGGER"; //Adds tag to show where logging is coming from

bool SDLogger::init() {
    // 1. Set up SPI communication with the SD card
    spi_bus_config_t bus = {}; //make bus

    //bus characteristics 
    bus.mosi_io_num = SD_MOSI_PIN;  //Sends data from ESP32 to SD card
    bus.miso_io_num = SD_MISO_PIN; //Sends data from SD card to ESP32
    bus.sclk_io_num = SD_CLK_PIN; //Provides clock pulses to synchronize communication
    // bus.quadwp_io_num = -1;
    // bus.quadhd_io_num = -1;
    bus.max_transfer_sz = 4096;

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &bus, SDSPI_DEFAULT_DMA);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI initialization failed");
        return false; //check SPI initialization
    }

    // 2. Configure SD card
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;

    sdspi_device_config_t slot = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot.gpio_cs = (gpio_num_t)SD_CS_PIN;
    slot.host_id = SPI2_HOST;

    // 3. Mount SD card
    esp_vfs_fat_mount_config_t mount = {};
    mount.format_if_mount_failed = false;
    mount.max_files = 5;
    mount.allocation_unit_size = 16 * 1024;

    ret = esp_vfs_fat_sdspi_mount(
        SD_MOUNT_POINT, &host, &slot, &mount, &card
    );

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SD card mount failed");
        spi_bus_free(SPI2_HOST);
        return false; //Check sd card mount
    }

    mounted = true;

    // 4. Check if CSV already exists
    FILE* existing = fopen("/sdcard/log.csv", "r");
    bool new_file = (existing == nullptr);

    if (existing != nullptr) {
        fclose(existing);
    }

    // 5. Open CSV for writing
    log_file = fopen("/sdcard/log.csv", "a");

    if (log_file == nullptr) {
        ESP_LOGE(TAG, "Failed to open CSV");
        close();
        return false;
    }

    // 6. Write header if this is a new file
    if (new_file) {
        if (fprintf(log_file,
            "time_ms,shock_rl_mm,shock_rr_mm,ax,ay,az,qw,qi,qj,qk\n") < 0 ||
            fflush(log_file) != 0) {
            ESP_LOGE(TAG, "Failed to write CSV header");
            close();
            return false;
        }
    }

    ESP_LOGI(TAG, "SD card ready");
    return true;
}


// Write one row of sensor data to the CSV
bool SDLogger::write(const char* data) {
    if (log_file == nullptr) {
        return false;
    }

    if (fprintf(log_file, "%s\n", data) < 0) {
        ESP_LOGE(TAG, "CSV write failed");
        return false;
    }

    return true;
}


// Close the file and unmount the SD card
void SDLogger::close() {
    if (log_file != nullptr) {
        fflush(log_file);
        fclose(log_file);
        log_file = nullptr;
    }

    if (mounted) {
        esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, card);
        mounted = false;
        card = nullptr;
    }
}

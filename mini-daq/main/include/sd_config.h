
#ifndef SD_CONFIG_H
#define SD_CONFIG_H

// SD card SPI pin configuration
constexpr int SD_MISO_PIN = 9;
constexpr int SD_CLK_PIN  = 10;
constexpr int SD_MOSI_PIN = 11;
constexpr int SD_CS_PIN   = 12;
constexpr int SD_DET_PIN  = 7;

// SD card mount location
constexpr const char* SD_MOUNT_POINT = "/sdcard";

#endif
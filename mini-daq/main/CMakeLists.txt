
idf_component_register(
    SRCS
        "app_main.cpp"
        "src/sd_logger.cpp"
        "../../daq/main/src/sensors/shock_pot_sensor.cpp"
        "../../daq/main/src/sensors/imu_sensor.cpp"

    INCLUDE_DIRS
        "include"
        "../../daq/main/include"

    REQUIRES
        fatfs
        sdmmc
        esp_driver_sdspi
        esp_driver_spi
        esp_driver_gpio
        esp_adc
        esp_timer
        common
        esp32_BNO08x
)

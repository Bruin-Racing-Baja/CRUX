idf_component_register(
    SRCS
        "src/sensors/shock_pot_sensor.cpp"
        "src/sensors/gps_sensor.cpp"
        "src/sensors/imu_sensor.cpp"
        "src/sensors/brake_pressure_sensor.cpp"
        "app_main.cpp"

    INCLUDE_DIRS
        "include"

    REQUIRES
        common
        esp_driver_gpio
        esp_adc
        esp_timer
        driver
        esp32_BNO08x
)
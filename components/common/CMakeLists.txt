idf_component_register(
    SRCS
        "src/gpio_wrapper.cpp"
        "src/telemetry.cpp"

    INCLUDE_DIRS
        "include"
    REQUIRES
        esp_driver_gpio
        esp_adc
        driver
        esp_timer
)
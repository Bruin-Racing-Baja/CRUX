idf_component_register(
    SRCS
        "src/gpio_wrapper.cpp"
        "src/telemetry.cpp"

    INCLUDE_DIRS
        "include"
    REQUIRES
        esp_driver_gpio
        esp_adc
        esp_driver_uart
        esp_driver_usb_serial_jtag
        esp_timer
)

idf_component_register(
    SRCS 
        "src/ecvt_controller.cpp" 
        "src/centerlock_controller.cpp" 
        "src/odrive.cpp"
        "src/gpio_wrapper.cpp"
        "src/telemetry.cpp"
        "src/sensors/gear_tooth_sensor.cpp" 
        "src/shift_register.cpp"
        "app_main.cpp"

    INCLUDE_DIRS 
        "include"

    REQUIRES
        esp_driver_twai
        driver 
        esp_adc 
        esp_timer
)



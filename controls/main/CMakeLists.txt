
idf_component_register(
    SRCS 
        "src/ecvt_controller.cpp" 
        "src/centerlock_controller.cpp" 
        "src/odrive.cpp"
        "src/sensors/gear_tooth_sensor.cpp" 
        "src/shift_register.cpp"
        "src/iir_filter.cpp"
        "src/median_filter.cpp"
        "app_main.cpp"

    INCLUDE_DIRS 
        "include"

    REQUIRES
        common
        esp_driver_twai
        driver 
        esp_adc 
        esp_timer
)



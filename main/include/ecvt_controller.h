#ifndef ECVT_CONTROLLER_H
#define ECVT_CONTROLLER_H

#include <sensors/gear_tooth_sensor.h>
#include <input_output/shift_register.h>
#include <odrive.h> 
#include <constants.h>

#include "esp_timer.h"
typedef enum {
    NORMAL = 0, 
    BUTTON_SHIFT = 1
} controller_mode_t;

class ECVTController {
public:
    ECVTController(controller_mode_t mode_, ShiftRegister* sr, bool wait_for_can = true);

    void init(bool wait_for_can);
    bool home_actuator(); 
    
    void control_function(); 
    void button_shift_control_function(); 
    void normal_control_function(); 

private: 
    static ECVTController* instance;
    controller_mode_t mode; 

    static IRAM_ATTR void primary_isr(void* p = nullptr);
    static IRAM_ATTR void secondary_isr(void* p = nullptr);
    static IRAM_ATTR void outbound_isr(void* p = nullptr);
    static IRAM_ATTR void inbound_isr(void* p = nullptr);

    GearToothSensor primary_gts; 
    GearToothSensor secondary_gts; 

    ODrive odrive;

    ShiftRegister* shift_reg;

    uint64_t control_cycle_count;
};

#endif // ECVT_CONTROLLER_H
#ifndef ECVT_CONTROLLER_H
#define ECVT_CONTROLLER_H
#include <odrive.h>
#include <sys/time.h>
#include <telemetry.h>
#include "esp_timer.h"
#include <sensors/gear_tooth_sensor.h>
#include <filters/iir_filter.h>
#include <filters/median_filter.h>
#include <constants.h>
#include <input_output/shift_register.h>
#include <odrive.h> 

class ECVTController 
{
public:
    ECVTController(ShiftRegister* sr, bool wait_for_can = true);
    void init(bool wait_for_can=true);

private:
    bool home_actuator(uint32_t timeout_ms=3000);
    void control_loop();

    bool get_outbound_limit(); 
    bool get_inbound_limit(); 
    bool get_engage_limit();

    static IRAM_ATTR void primary_isr(void* p = nullptr);
    static IRAM_ATTR void secondary_isr(void* p = nullptr);
    static IRAM_ATTR void outbound_isr(void* p = nullptr);
    static IRAM_ATTR void inbound_isr(void* p = nullptr);

    static void timerCallback(void* arg);
    static void taskWrapper(void* pvParameters);

    static ECVTController* instance;

    GearToothSensor primary_gts; 
    GearToothSensor secondary_gts; 

    ODrive odrive;
    ShiftRegister* shift_reg;
    Telemetry* telem;

    uint64_t control_cycle_count;
    float last_engine_rpm_error;
    float actuator_engage_position;

    TaskHandle_t taskHandle;
    esp_timer_handle_t timerHandle;
   
    IIRFilter engine_rpm_rotation_filter{
        ENGINE_RPM_ROTATION_FILTER_B, ENGINE_RPM_ROTATION_FILTER_A,
        ENGINE_RPM_ROTATION_FILTER_M, ENGINE_RPM_ROTATION_FILTER_N
    };

    IIRFilter engine_rpm_time_filter{
        ENGINE_RPM_TIME_FILTER_B, ENGINE_RPM_TIME_FILTER_A,
        ENGINE_RPM_TIME_FILTER_M, ENGINE_RPM_TIME_FILTER_N
    };

    IIRFilter engine_rpm_derror_filter{
        ENGINE_RPM_DERROR_FILTER_B, ENGINE_RPM_DERROR_FILTER_A,
        ENGINE_RPM_DERROR_FILTER_M, ENGINE_RPM_DERROR_FILTER_N
    };

    IIRFilter gear_rpm_time_filter{
        GEAR_RPM_TIME_FILTER_B, GEAR_RPM_TIME_FILTER_A,
        GEAR_RPM_TIME_FILTER_M, GEAR_RPM_TIME_FILTER_N
    };

    IIRFilter throttle_filter{ 
        THROTTLE_FILTER_B, THROTTLE_FILTER_A,
        THROTTLE_FILTER_M, THROTTLE_FILTER_N
    };

    MedianFilter engine_rpm_median_filter{ENGINE_RPM_MEDIAN_FILTER_WINDOW};
};

#endif // ECVT_CONTROLLER_H
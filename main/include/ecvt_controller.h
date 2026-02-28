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

class ECVTController {
public:
    ECVTController(ODrive* odrive, GearToothSensor* primary_sensor, GearToothSensor* secondary_sensor) : ecvt_odrive(odrive), primary_sensor(primary_sensor), secondary_sensor(secondary_sensor) {}
    void start();

private:
    TaskHandle_t taskHandle;
    esp_timer_handle_t timerHandle;
    ODrive* ecvt_odrive;
    void control_loop();
    Telemetry* telem;
    static void timerCallback(void* arg);
    static void taskWrapper(void* pvParameters);
    GearToothSensor* primary_sensor;
    GearToothSensor* secondary_sensor;
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

    IIRFilter throttle_filter{ // Fixed typo: throttle_fitler -> throttle_filter
        THROTTLE_FILTER_B, THROTTLE_FILTER_A,
        THROTTLE_FILTER_M, THROTTLE_FILTER_N
    };

    MedianFilter engine_rpm_median_filter{ENGINE_RPM_MEDIAN_FILTER_WINDOW};
    float last_engine_rpm_error;

};

#endif // ECVT_CONTROLLER_H
#include "ecvt_controller.h"

void ECVTController::control_loop()
{
    //This is just for testing right now
    if(telem->lock())
    {
        struct timeval tv_now;
        gettimeofday(&tv_now, NULL);
        int64_t time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
        telem->data.timestamp = (float) time_us / 1e6;
        telem->data.steer_angle = 0.0f;
        telem->data.battery_voltage = 1.0f;
        telem->unlock();

    }
    
}
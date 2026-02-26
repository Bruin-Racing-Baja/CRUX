#include "ecvt_controller.h"
#include "esp_log.h"

#include <macros.h>

static const char *TAG = "twai_sender";
void ECVTController::start()
{
    xTaskCreatePinnedToCore(taskWrapper, "ecvt_task", 4096, this, 10, &taskHandle, 1);

    const esp_timer_create_args_t timer_args = {
        .callback = &timerCallback,
        .arg = this,
        .name = "ecvt_timer"
    };
    esp_timer_create(&timer_args, &timerHandle);
    esp_timer_start_periodic(timerHandle, 10000);
}

void ECVTController::timerCallback(void* arg) {
    ECVTController* controller = (ECVTController*)arg;
    vTaskNotifyGiveFromISR(controller->taskHandle, NULL);
}
void ECVTController::taskWrapper(void* pvParameters) {
    ((ECVTController*)pvParameters)->control_loop();
}
void ECVTController::control_loop()
{
    ESP_LOGI(TAG, "Start");
    uint8_t node_id = 3;
    float dt_s = CONTROL_FUNCTION_INTERVAL_MS * SECONDS_PER_MS;
    float target_rpm = 3000;
    while(true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);   
        float primary_rpm = primary_sensor->get_rpm();
        float secondary_rpm = secondary_sensor->get_rpm();
        
        primary_rpm = engine_rpm_median_filter.update(primary_rpm);
        primary_rpm = engine_rpm_time_filter.update(primary_rpm);

        secondary_rpm = gear_rpm_time_filter.update(secondary_rpm);
        secondary_rpm = secondary_rpm / GEAR_TO_SECONDARY_RATIO;

        float engine_rpm_error = primary_rpm - target_rpm;
        float filtered_engine_rpm_error = engine_rpm_derror_filter.update(engine_rpm_error);

        float engine_rpm_derror =
            (filtered_engine_rpm_error - last_engine_rpm_error) / dt_s;
        last_engine_rpm_error = filtered_engine_rpm_error;


        float velocity_command =
            (engine_rpm_error * ACTUATOR_KP +
            MAX(0, engine_rpm_derror * ACTUATOR_KD));
        velocity_command = CLAMP(velocity_command, -10.f, 10.f);
        ecvt_odrive->set_axis_state(node_id, AXIS_STATE_CLOSED_LOOP_CONTROL);
        ecvt_odrive->set_input_vel( node_id, velocity_command, 0.0f);


        
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
    
    
}
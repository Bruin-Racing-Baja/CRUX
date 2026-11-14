#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include <atomic>

#include "esp_vfs_dev.h"
#include "esp_vfs_usb_serial_jtag.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"

#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <constants.h> 
#include <gpio_wrapper.h>
#include <odrive.h>
#include <telemetry.h>
#include <ecvt_controller.h>
#include <centerlock_controller.h>

#include <sensors/gear_tooth_sensor.h>
#include <sensors/brake_pot_sensor.h>
#include <sensors/throt_pot_sensor.h>

#include <input_output/shift_register.h>
#include <input_output/centerlock_limit_switch.h>
#include <input_output/button.h>
#include <input_output/led.h>

static const char *TAG = "twai_sender";

/* Globally Defined For Now */
CenterlockLimitSwitch centerlock_ls(CENTERLOCK_LIMIT_SWITCH_OUTBOUND_PIN, CENTERLOCK_LIMIT_SWITCH_INBOUND_PIN); 
ShiftRegister shift_reg(SR_SER_IN_PIN, SR_SHIFT_REG_CLK_PIN, SR_REG_CLK_PIN); 
ECVTController ecvt_controller(&shift_reg);
CenterlockController centerlock_controller(CENTERLOCK_LIMIT_SWITCH_INBOUND_PIN, CENTERLOCK_LIMIT_SWITCH_INBOUND_PIN, CENTERLOCK_LED_PIN); 

// ODrive odrive(4); 
Button button_1(BUTTON_1_PIN);
Button button_2(BUTTON_2_PIN);
Button button_3(BUTTON_3_PIN);
Button button_4(BUTTON_4_PIN);

// Log override button
std::atomic<bool> log_override_on{false};

// Transmission completion callback
static IRAM_ATTR bool twai_sender_tx_done_callback(twai_node_handle_t handle, const twai_tx_done_event_data_t *edata, void *user_ctx)
{
    if (!edata->is_tx_success) {
        ESP_EARLY_LOGW(TAG, "Failed to transmit message, ID: 0x%X", edata->done_tx_frame->header.id);
    }
    return false; // No task wake required
}

// Bus error callback
static IRAM_ATTR bool twai_sender_on_error_callback(twai_node_handle_t handle, const twai_error_event_data_t *edata, void *user_ctx)
{
    return false; // No task wake required
}

static void log_button_task(void*) {
    bool last_pressed = false;
    int64_t last_toggle_us = 0;
    for (;;) {
        bool pressed = (gpio_get_level(BUTTON_1_PIN) == 0);   // active-low
        int64_t now = esp_timer_get_time();
        if (pressed && !last_pressed && (now - last_toggle_us) > 50000) {
            log_override_on.store(!log_override_on.load());
            last_toggle_us = now;
        }
        last_pressed = pressed;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

extern "C" void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(500));

    Telemetry::init();

    // log override config
    gpio_config_t btn_cfg = {};
    btn_cfg.pin_bit_mask = 1ULL << BUTTON_1_PIN;
    btn_cfg.mode = GPIO_MODE_INPUT;
    btn_cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&btn_cfg);
    xTaskCreate(log_button_task, "log_btn", 2048, nullptr, 3, nullptr);

    vTaskDelay(pdMS_TO_TICKS(500));
    xTaskCreatePinnedToCore(Telemetry::send_data, "telemetry_task", 4096,  nullptr, tskIDLE_PRIORITY + 5, NULL, 0);
    ODrive::init(CAN_TX_PIN, CAN_RX_PIN, CAN_BITRATE);
    ODrive::start();
    ecvt_controller.init(true);
    //centerlock_controller.init();

    while(true)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

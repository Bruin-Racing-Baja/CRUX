#include "esp_vfs_dev.h"
#include "esp_vfs_usb_serial_jtag.h"
#include "driver/usb_serial_jtag.h"
#include <stdio.h>
#include "driver/usb_serial_jtag.h"
#include <string.h>
#include <sys/param.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "driver/gpio.h"

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

controller_mode_t ecvt_mode = NORMAL; 

/* Globally Defined For Now */
CenterlockLimitSwitch centerlock_ls(CENTERLOCK_LIMIT_SWITCH_OUTBOUND_PIN, CENTERLOCK_LIMIT_SWITCH_INBOUND_PIN); 
ShiftRegister shift_reg(SR_SER_IN_PIN, SR_SHIFT_REG_CLK_PIN, SR_REG_CLK_PIN); 
ECVTController ecvt_controller(ecvt_mode, &shift_reg);
CenterlockController centerlock_controller(CENTERLOCK_LIMIT_SWITCH_INBOUND_PIN, CENTERLOCK_LIMIT_SWITCH_INBOUND_PIN, CENTERLOCK_LED_PIN); 

// ODrive odrive(4); 
Button button_1(BUTTON_1_PIN);
Button button_2(BUTTON_2_PIN);
Button button_3(BUTTON_3_PIN);
Button button_4(BUTTON_4_PIN);

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
    //ESP_EARLY_LOGW(TAG2, "TWAI node error: 0x%x", edata->err_flags.val);
    return false; // No task wake required
}

extern "C" void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(500));

    Telemetry::init();

    vTaskDelay(pdMS_TO_TICKS(500));
    xTaskCreatePinnedToCore(Telemetry::send_data, "telemetry_task", 4096,  nullptr, tskIDLE_PRIORITY + 5, NULL, 0);
    ecvt_controller.init(true);
    centerlock_controller.init();

    while(true)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

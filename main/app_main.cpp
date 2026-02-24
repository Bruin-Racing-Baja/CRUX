#include <stdio.h>
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
#include <ecvt_controller.h>

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
// ECVTController ecvt_controller(ecvt_mode, &shift_reg);
ODrive odrive(4); 
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
    ESP_EARLY_LOGW(TAG, "TWAI node error: 0x%x", edata->err_flags.val);
    return false; // No task wake required
}

/* Callback for Centerlock Outbound */
static void IRAM_ATTR centerlock_ls_outbound_callback(void * params) {
    centerlock_ls.update_isr_outbound();
    odrive.set_input_vel(0);
}

/* Callback for Centerlock Inbound */
static void IRAM_ATTR centerlock_ls_inbound_callback(void * params) {
    centerlock_ls.update_isr_inbound(); 
    odrive.set_input_vel(0);
}

extern "C" void app_main(void)
{

    /* Initialize ODrive */
    odrive.init(CAN_TX_PIN, CAN_RX_PIN, CAN_BITRATE);
    odrive.start();
    odrive.clear_errors();

    bool wait_for_can = true; 
    /* Flash While Wait for CAN Heartbeat */
    shift_reg.write_all_leds(true);
    if (wait_for_can) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI(TAG, "time since heartbeat: %d", odrive.get_time_since_last_heartbeat()); 
        bool state = true; 
        while (odrive.get_time_since_last_heartbeat() > 1e6) {
            shift_reg.write_all_leds(state);
            vTaskDelay(pdMS_TO_TICKS(100));
            state = !state;
        }
        ESP_LOGI(TAG, "time since heartbeat: %d", odrive.get_time_since_last_heartbeat());
    }
    shift_reg.write_all_leds(false);

    vTaskDelay(pdMS_TO_TICKS(3000));
    odrive.set_axis_state(AXIS_STATE_CLOSED_LOOP_CONTROL); 
    vTaskDelay(pdMS_TO_TICKS(3000));
    odrive.set_controller_mode(CTRL_MODE_VELOCITY_CONTROL, INPUT_MODE_VEL_RAMP);
    vTaskDelay(pdMS_TO_TICKS(3000));
    odrive.set_input_vel(2); 
    vTaskDelay(pdMS_TO_TICKS(3000));
    odrive.set_input_vel(0); 
    
    while(true) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

}

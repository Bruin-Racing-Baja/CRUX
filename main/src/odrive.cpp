#include "odrive.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

static const char* TAG = "ODrive";
twai_node_handle_t ODrive::node_handle_ = nullptr;
ODrive::RxFrameBuffer* ODrive::rx_pool_ = nullptr;
volatile int ODrive::write_idx_ = 0;
volatile int ODrive::read_idx_ = 0;
SemaphoreHandle_t ODrive::free_pool_sem_ = nullptr;
SemaphoreHandle_t ODrive::rx_ready_sem_ = nullptr;
TaskHandle_t ODrive::rx_task_handle_ = nullptr;
volatile bool ODrive::running_ = false;
ODrive* ODrive::ecvt_instance = nullptr;
ODrive* ODrive::centerlock_instance = nullptr;


QueueHandle_t ODrive::can_tx_queue; 
ODrive::ODrive(uint8_t node_id)
    :  last_heartbeat_us(0)
    , node_id_(node_id)
{
}

ODrive::~ODrive()
{
    stop();
}

bool ODrive::init(gpio_num_t tx_pin, gpio_num_t rx_pin, uint32_t bitrate)
{
    rx_pool_ = (RxFrameBuffer*)calloc(RX_BUFFER_DEPTH, sizeof(RxFrameBuffer));
    if (!rx_pool_) {
        ESP_LOGE(TAG, "Failed to allocate RX buffer pool");
        return false;
    }

    // Initialize frame buffers
    for (int i = 0; i < RX_BUFFER_DEPTH; i++) {
        rx_pool_[i].frame.buffer = rx_pool_[i].data;
        rx_pool_[i].frame.buffer_len = sizeof(rx_pool_[i].data);
    }

    free_pool_sem_ = xSemaphoreCreateCounting(RX_BUFFER_DEPTH, RX_BUFFER_DEPTH);
    rx_ready_sem_ = xSemaphoreCreateCounting(RX_BUFFER_DEPTH, 0);

    if (!free_pool_sem_ || !rx_ready_sem_) {
        ESP_LOGE(TAG, "Failed to create semaphores");
        free(rx_pool_);
        rx_pool_ = nullptr;
        return false;
    }
    twai_onchip_node_config_t node_config = {
        .io_cfg = {
            .tx = tx_pin,
            .rx = rx_pin,
            .quanta_clk_out = GPIO_NUM_NC,
            .bus_off_indicator = GPIO_NUM_NC,
        },
        .bit_timing = {
            .bitrate = bitrate,
        },
        .fail_retry_cnt = 3,
        .tx_queue_depth = TX_QUEUE_DEPTH,
        .flags = {
            .enable_self_test = false,
            .enable_loopback = false,
        },
    };
    

    twai_mask_filter_config_t mfilter_cfg = {
    .id = 0x00,         
    .mask = 0x00,                          
    .is_ext = false,    
    };
    // Create TWAI node
    
    ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &node_handle_));
    
    ESP_ERROR_CHECK(twai_node_config_mask_filter(node_handle_, 0, &mfilter_cfg));   // Configure on filter 0
    twai_event_callbacks_t callbacks = {};
    callbacks.on_rx_done = ODrive::on_rx_done_ISR;
    callbacks.on_error = ODrive::on_error_ISR;
    callbacks.on_state_change = ODrive::on_state_change_ISR;

    ESP_ERROR_CHECK(twai_node_register_event_callbacks(node_handle_, &callbacks, nullptr));
    /*
    ESP_LOGI(TAG, "TWAI node created (TX: GPIO%d, RX: GPIO%d, %lu bps, %d buffer depth)", 
             tx_pin, rx_pin, bitrate, RX_BUFFER_DEPTH);
    */
    can_tx_queue = xQueueCreate(50, sizeof(CanMessage));
    return true;
}

bool ODrive::start()
{
    ESP_ERROR_CHECK(twai_node_enable(node_handle_));
    running_ = true;

    BaseType_t result = xTaskCreatePinnedToCore(
        rx_task,
        "odrive_can_rx",
        4096,
        nullptr,
        tskIDLE_PRIORITY + 6,
        &rx_task_handle_,
        1
    );

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create RX task");
        twai_node_disable(node_handle_);
        running_ = false;
        return false;
    }
    xTaskCreatePinnedToCore(can_tx_task, "odrive_can_tx", 4096, nullptr, tskIDLE_PRIORITY + 6, NULL, 1);
    //ESP_LOGI(TAG, "ODrive CAN started");
    return true;
}

void ODrive::stop()
{
    running_ = false;
    if (rx_ready_sem_) {
        xSemaphoreGive(rx_ready_sem_);
    }

    // Wait a bit for task to exit
    if (rx_task_handle_) {
        vTaskDelay(pdMS_TO_TICKS(200));
        vTaskDelete(rx_task_handle_);
        rx_task_handle_ = nullptr;
    }

    // Disable and delete TWAI node
    if (node_handle_) {
        twai_node_disable(node_handle_);
        twai_node_delete(node_handle_);
        node_handle_ = nullptr;
    }

    // Clean up semaphores
    if (free_pool_sem_) {
        vSemaphoreDelete(free_pool_sem_);
        free_pool_sem_ = nullptr;
    }
    if (rx_ready_sem_) {
        vSemaphoreDelete(rx_ready_sem_);
        rx_ready_sem_ = nullptr;
    }

    // Free buffer pool
    if (rx_pool_) {
        free(rx_pool_);
        rx_pool_ = nullptr;
    }

    ESP_LOGI(TAG, "ODrive CAN stopped");
}

bool IRAM_ATTR ODrive::on_rx_done_ISR(twai_node_handle_t handle, const twai_rx_done_event_data_t* edata, void* user_ctx)
{
    BaseType_t woken = pdFALSE;

    // Check if we have free buffer slots
    if (xSemaphoreTakeFromISR(free_pool_sem_, &woken) != pdTRUE) {
        ESP_EARLY_LOGW(TAG, "RX buffer full, dropping frame");
        return (woken == pdTRUE);
    }

    // Receive frame into ring buffer
    if (twai_node_receive_from_isr(handle, &rx_pool_[write_idx_].frame) == ESP_OK) {
        write_idx_ = (write_idx_ + 1) % RX_BUFFER_DEPTH;
        xSemaphoreGiveFromISR(rx_ready_sem_, &woken);
    } else {
        // Failed to receive, give back the free slot
        xSemaphoreGiveFromISR(free_pool_sem_, &woken);
    }

    return (woken == pdTRUE);
}

bool IRAM_ATTR ODrive::on_error_ISR(twai_node_handle_t handle, const twai_error_event_data_t* edata, void* user_ctx)
{
    ESP_EARLY_LOGW(TAG, "CAN bus error: 0x%lx", edata->err_flags.val);
    return false;
}

bool IRAM_ATTR ODrive::on_state_change_ISR(twai_node_handle_t handle, const twai_state_change_event_data_t* edata, void* user_ctx)
{
    const char* state_names[] = {"ERROR_ACTIVE", "ERROR_WARNING", "ERROR_PASSIVE", "BUS_OFF"};
    ESP_EARLY_LOGI(TAG, "State: %s -> %s", state_names[edata->old_sta], state_names[edata->new_sta]);
    return false;
}

void ODrive::can_tx_task(void* pvParameters) {
    CanMessage msg;
    
    static uint8_t tx_data[8];
    static twai_frame_t tx_frame = {};
    
    tx_frame.buffer = tx_data;
    tx_frame.header.ide = false; 
    tx_frame.header.rtr = false; 
    tx_frame.header.fdf = false; 

    while (true) {
        if (xQueueReceive(can_tx_queue, &msg, portMAX_DELAY) == pdTRUE) {
            
            tx_frame.header.id = msg.id;
            tx_frame.header.dlc = msg.len;
            tx_frame.buffer_len = msg.len;
            memcpy(tx_data, msg.data, msg.len);

            if (twai_node_transmit(node_handle_, &tx_frame, pdMS_TO_TICKS(100)) == ESP_OK) {
                
                // CRITICAL FIX: Wait for transmission to finish before we loop 
                // around and overwrite tx_data with the next message!
                twai_node_transmit_wait_all_done(node_handle_, pdMS_TO_TICKS(100));
            }
        }
    }
}


void ODrive::rx_task(void* arg)
{
    ESP_LOGI(TAG, "RX task started");

    while (running_) {
        // Block until frame is available (or timeout for periodic checks)
        if (xSemaphoreTake(rx_ready_sem_, pdMS_TO_TICKS(1000)) == pdTRUE) {
            // Process the frame
            twai_frame_t* frame = &rx_pool_[read_idx_].frame;
            process_msg(*frame);

            // Move to next slot and mark as free
            read_idx_ = (read_idx_ + 1) % RX_BUFFER_DEPTH;
            xSemaphoreGive(free_pool_sem_);
        }
    }

    ESP_LOGI(TAG, "RX task exiting");
    vTaskDelete(NULL);
    
}

uint32_t ODrive::build_can_id(uint16_t cmd_id)
{
    // ODrive CAN ID format: (node_id_ << 5) | cmd_id
    //ESP_LOGI(TAG, "can_id: %d", ((uint32_t)node_id_ << 5) | (cmd_id & 0x1F));
    return ((uint32_t)node_id_ << 5) | (cmd_id & 0x1F);
}

void ODrive::send_can_msg(uint32_t can_id, const uint8_t* data, uint8_t len, bool remote)
{
    CanMessage msg = {};
    msg.id = can_id;
    msg.len = len;
    if (data && len > 0) {
        memcpy(msg.data, data, len);
    }
    if (can_tx_queue != nullptr) {
        xQueueSend(can_tx_queue, &msg, 0); 
    }
}

void ODrive::set_axis_state(odrive_axis_state_t state)
{
    uint32_t can_id = build_can_id(CAN_SET_AXIS_STATE);
    uint32_t state_val = (uint32_t)state;
    send_can_msg(can_id, (uint8_t*)&state_val, 4);
    //ESP_LOGI(TAG, "Set axis state: node=%d, state=%d", state);
}

void ODrive::set_controller_mode(odrive_control_mode_t ctrl_mode, odrive_input_mode_t input_mode)
{
    uint32_t can_id = build_can_id(CAN_SET_CONTROLLER_MODE);
    uint8_t data[8];
    memcpy(&data[0], &ctrl_mode, 4);
    memcpy(&data[4], &input_mode, 4);
    send_can_msg(can_id, data, 8);
    //ESP_LOGI(TAG, "Set controller mode: node=%d, ctrl=%d, input=%d", ctrl_mode, input_mode);
}

void ODrive::set_input_pos(float pos, int16_t vel_ff, int16_t torque_ff)
{
    uint32_t can_id = build_can_id(CAN_SET_INPUT_POS);
    uint8_t data[8];
    memcpy(&data[0], &pos, 4);
    memcpy(&data[4], &vel_ff, 2);
    memcpy(&data[6], &torque_ff, 2);
    send_can_msg(can_id, data, 8);
}

void ODrive::set_input_vel(float vel, float torque_ff)
{
    uint32_t can_id = build_can_id(CAN_SET_INPUT_VEL);
    uint8_t data[8];
    memcpy(&data[0], &vel, 4);
    memcpy(&data[4], &torque_ff, 4);
    send_can_msg(can_id, data, 8);
    ESP_EARLY_LOGD(TAG, "Set input velocity: node=%d, vel=%.2f, torque_ff=%.2f", node_id_, vel, torque_ff);
}

void ODrive::set_input_torque(float torque)
{
    uint32_t can_id = build_can_id(CAN_SET_INPUT_TORQUE);
    send_can_msg(can_id, (uint8_t*)&torque, 4);
}

void ODrive::set_limits(float vel_limit, float current_limit)
{
    uint32_t can_id = build_can_id(CAN_SET_LIMITS);
    uint8_t data[8];
    memcpy(&data[0], &vel_limit, 4);
    memcpy(&data[4], &current_limit, 4);
    send_can_msg(can_id, data, 8);
    //ESP_LOGI(TAG, "Set limits: node=%d, vel=%.2f, current=%.2f", vel_limit, current_limit);
}

void ODrive::set_pos_gain(float pos_gain)
{
    uint32_t can_id = build_can_id(CAN_SET_POS_GAIN);
    send_can_msg(can_id, (uint8_t*)&pos_gain, 4);
}

void ODrive::set_vel_gains(float vel_gain, float vel_integrator_gain)
{
    uint32_t can_id = build_can_id(CAN_SET_VEL_GAINS);
    uint8_t data[8];
    memcpy(&data[0], &vel_gain, 4);
    memcpy(&data[4], &vel_integrator_gain, 4);
    send_can_msg(can_id, data, 8);
}

void ODrive::set_absolute_position(float pos)
{
    uint32_t can_id = build_can_id(CAN_SET_ABSOLUTE_POSITION);
    send_can_msg(can_id, (uint8_t*)&pos, 4);
}

void ODrive::estop()
{
    uint32_t can_id = build_can_id(CAN_ESTOP);
    send_can_msg(can_id, nullptr, 0);
    ESP_LOGW(TAG, "E-STOP sent to node %d", node_id_);
}

void ODrive::clear_errors()
{
    uint32_t can_id = build_can_id(CAN_CLEAR_ERRORS);
    send_can_msg(can_id, nullptr, 0);
    //ESP_LOGI(TAG, "Clear errors: node=%d", node_id_);
}

void ODrive::reboot()
{
    uint32_t can_id = build_can_id(CAN_REBOOT);
    send_can_msg(can_id, nullptr, 0);
    //ESP_LOGI(TAG, "Reboot: node=%d", node_id_);
}

void ODrive::request_encoder_est()
{
    uint32_t can_id = build_can_id(CAN_GET_ENCODER_ESTIMATES);
    send_can_msg(can_id, nullptr, 0);
}

void ODrive::request_iq()
{
    uint32_t can_id = build_can_id(CAN_GET_IQ);
    send_can_msg(can_id, nullptr, 0);
}

void ODrive::request_bus_voltage_current()
{
    uint32_t can_id = build_can_id(CAN_GET_BUS_VOLTAGE_CURRENT);
    send_can_msg(can_id, nullptr, 0);
}

void ODrive::request_temperature()
{
    uint32_t can_id = build_can_id(CAN_GET_TEMPERATURE);
    send_can_msg(can_id, nullptr, 0);
}

void ODrive::request_total_charge_used()
{
    uint32_t can_id = build_can_id(CAN_RXSDO);
    uint8_t data[8] = {0};
    memcpy(data + 1, &TOTAL_CHARGE_USED_ID, 2);
    send_can_msg(can_id, data, 8);
}

void ODrive::request_total_power_used()
{
    uint32_t can_id = build_can_id(CAN_RXSDO);
    uint8_t data[8] = {0};
    memcpy(data + 1, &TOTAL_POWER_USED, 2);
    send_can_msg(can_id, data, 8);
}


uint32_t ODrive::get_time_since_last_heartbeat() {
    return esp_timer_get_time() - last_heartbeat_us;
}

void ODrive::process_msg(const twai_frame_t& msg)
{
    // Extract node ID and command ID from CAN ID
    uint8_t node_id_ = (msg.header.id >> 5) & 0x3F;
    uint16_t cmd_id = msg.header.id & 0x1F;
    /*
    ESP_LOGI(TAG, "RX: %x [%d] %x %x %x %x", \
                  msg.header.id, msg.header.dlc, msg.buffer[0], msg.buffer[1], msg.buffer[2], msg.buffer[3]);
   */      
    ODrive* instance = nullptr;
    if (ecvt_instance && node_id_ == ecvt_instance->node_id_) {
        instance = ecvt_instance;
    } else if (centerlock_instance && node_id_ == centerlock_instance->node_id_) {
        instance = centerlock_instance;
    } else {
        // Unknown node ID, ignore
        return;
    }

    switch (cmd_id) {
        case CAN_HEARTBEAT:
            instance->parse_heartbeat(msg.buffer, msg.header.dlc);
            instance->last_heartbeat_us = esp_timer_get_time();
            break;
            
        case CAN_GET_ENCODER_ESTIMATES:
            instance->parse_encoder_estimates(msg.buffer, msg.header.dlc);
            break;
            
        case CAN_GET_IQ:
            instance->parse_iq(msg.buffer, msg.header.dlc);
            break;
        case CAN_GET_BUS_VOLTAGE_CURRENT:
            instance->parse_bus_voltage_current(msg.buffer, msg.header.dlc);
            break;

        case CAN_TXSDO:
            uint16_t endpoint_id;
            memcpy(&endpoint_id, msg.buffer + 1, 2);
            if (endpoint_id == TOTAL_CHARGE_USED_ID) 
                memcpy(&instance->total_charge_used, msg.buffer + 4, 4);
            
            else if (endpoint_id == TOTAL_POWER_USED) 
                memcpy(&instance->total_power_used, msg.buffer + 4, 4);
            
            
        default:
            // Unhandled message type
            break;
    }
}

void ODrive::parse_heartbeat(const uint8_t* data, uint8_t len)
{
    if (len < 8) return;
    
    uint32_t error;
    uint8_t state;
    
    memcpy(&error, &data[0], 4);
    state = data[4];
    // data[5-7] contain procedure result and trajectory done flag
    
}

void ODrive::parse_encoder_estimates(const uint8_t* data, uint8_t len)
{
    if (len < 8) return;
    
    memcpy(&pos, &data[0], 4);
    memcpy(&vel, &data[4], 4);
}

float ODrive::get_pos()
{
    return pos;
}

float ODrive::get_vel()
{
    return vel;
}

float ODrive::get_iq()
{
    return iq_measured;
}

float ODrive::get_bus_voltage()
{
    return bus_voltage;
}

float ODrive::get_bus_current()
{
    return bus_current;
}

float ODrive::get_total_charge_used()
{
    return total_charge_used;
}

float ODrive::get_total_power_used()
{
    return total_power_used;
}

void ODrive::parse_iq(const uint8_t* data, uint8_t len)
{
    if (len < 8) return;
    
    memcpy(&iq_setpoint, &data[0], 4);
    memcpy(&iq_measured, &data[4], 4); 
}

void ODrive::parse_bus_voltage_current(const uint8_t* data, uint8_t len)
{
    if (len < 8) return;
    
    memcpy(&bus_voltage, &data[0], 4);
    memcpy(&bus_current, &data[4], 4); 
}
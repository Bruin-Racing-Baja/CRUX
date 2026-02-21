#include "odrive.h"
#include "esp_log.h"
#include <string.h>

static const char* TAG = "ODrive";

ODrive::ODrive()
    : tx_pin_(GPIO_NUM_NC)
    , rx_pin_(GPIO_NUM_NC)
    , bitrate_(250000)
    , rx_buffer_depth_(64)
    , node_handle_(nullptr)
    , rx_pool_(nullptr)
    , write_idx_(0)
    , read_idx_(0)
    , free_pool_sem_(nullptr)
    , rx_ready_sem_(nullptr)
    , rx_task_handle_(nullptr)
    , running_(false)
    , heartbeat_cb_(nullptr)
    , heartbeat_ctx_(nullptr)
    , encoder_cb_(nullptr)
    , encoder_ctx_(nullptr)
    , iq_cb_(nullptr)
    , iq_ctx_(nullptr)
{
}

ODrive::~ODrive()
{
    stop();
}

bool ODrive::init(gpio_num_t tx_pin, gpio_num_t rx_pin, uint32_t bitrate)
{
    rx_pool_ = (RxFrameBuffer*)calloc(rx_buffer_depth_, sizeof(RxFrameBuffer));
    if (!rx_pool_) {
        ESP_LOGE(TAG, "Failed to allocate RX buffer pool");
        return false;
    }

    // Initialize frame buffers
    for (int i = 0; i < rx_buffer_depth_; i++) {
        rx_pool_[i].frame.buffer = rx_pool_[i].data;
        rx_pool_[i].frame.buffer_len = sizeof(rx_pool_[i].data);
    }

    free_pool_sem_ = xSemaphoreCreateCounting(rx_buffer_depth_, rx_buffer_depth_);
    rx_ready_sem_ = xSemaphoreCreateCounting(rx_buffer_depth_, 0);

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
        .tx_queue_depth = 10,
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

    ESP_ERROR_CHECK(twai_node_register_event_callbacks(node_handle_, &callbacks, this));

    ESP_LOGI(TAG, "TWAI node created (TX: GPIO%d, RX: GPIO%d, %lu bps, %d buffer depth)", 
             tx_pin, rx_pin, bitrate, rx_buffer_depth_);

    return true;
}

bool ODrive::start()
{
    ESP_ERROR_CHECK(twai_node_enable(node_handle_));
    running_ = true;

    BaseType_t result = xTaskCreate(
        rx_task_entry,
        "odrive_can_rx",
        4096,
        this,
        tskIDLE_PRIORITY + 5,
        &rx_task_handle_
    );

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create RX task");
        twai_node_disable(node_handle_);
        running_ = false;
        return false;
    }
    ESP_LOGI(TAG, "ODrive CAN started");
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
    ODrive* self = (ODrive*)user_ctx;
    BaseType_t woken = pdFALSE;

    // Check if we have free buffer slots
    if (xSemaphoreTakeFromISR(self->free_pool_sem_, &woken) != pdTRUE) {
        ESP_EARLY_LOGW(TAG, "RX buffer full, dropping frame");
        return (woken == pdTRUE);
    }

    // Receive frame into ring buffer
    if (twai_node_receive_from_isr(handle, &self->rx_pool_[self->write_idx_].frame) == ESP_OK) {
        self->write_idx_ = (self->write_idx_ + 1) % self->rx_buffer_depth_;
        xSemaphoreGiveFromISR(self->rx_ready_sem_, &woken);
    } else {
        // Failed to receive, give back the free slot
        xSemaphoreGiveFromISR(self->free_pool_sem_, &woken);
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

void ODrive::rx_task_entry(void* arg)
{
    ODrive* self = (ODrive*)arg;
    self->rx_task();
    vTaskDelete(NULL);
}

void ODrive::rx_task()
{
    ESP_LOGI(TAG, "RX task started");

    while (running_) {
        // Block until frame is available (or timeout for periodic checks)
        if (xSemaphoreTake(rx_ready_sem_, pdMS_TO_TICKS(1000)) == pdTRUE) {
            // Process the frame
            twai_frame_t* frame = &rx_pool_[read_idx_].frame;
            process_msg(*frame);

            // Move to next slot and mark as free
            read_idx_ = (read_idx_ + 1) % rx_buffer_depth_;
            xSemaphoreGive(free_pool_sem_);
        }
    }

    ESP_LOGI(TAG, "RX task exiting");
}

uint32_t ODrive::build_can_id(uint8_t node_id, uint16_t cmd_id)
{
    // ODrive CAN ID format: (node_id << 5) | cmd_id
    return ((uint32_t)node_id << 5) | (cmd_id & 0x1F);
}

void ODrive::send_can_msg(uint32_t can_id, const uint8_t* data, uint8_t len, bool remote)
{
    twai_frame_t tx_msg = {
        .header = {
            .id = can_id,
            .dlc = len,
            .ide = false,
            .rtr = remote,
            .fdf = false,
        },
        .buffer_len = len,  // Length of data to transmit
    };

    if (data && len > 0) {
        memcpy(tx_msg.buffer, data, len);
    }

    ESP_ERROR_CHECK(twai_node_transmit(node_handle_, &tx_msg, pdMS_TO_TICKS(100)));
    
}

void ODrive::set_axis_state(uint8_t node_id, odrive_axis_state_t state)
{
    uint32_t can_id = build_can_id(node_id, CAN_SET_AXIS_STATE);
    uint32_t state_val = (uint32_t)state;
    send_can_msg(can_id, (uint8_t*)&state_val, 4);
    ESP_LOGI(TAG, "Set axis state: node=%d, state=%d", node_id, state);
}

void ODrive::set_controller_mode(uint8_t node_id, odrive_control_mode_t ctrl_mode, odrive_input_mode_t input_mode)
{
    uint32_t can_id = build_can_id(node_id, CAN_SET_CONTROLLER_MODE);
    uint8_t data[8];
    memcpy(&data[0], &ctrl_mode, 4);
    memcpy(&data[4], &input_mode, 4);
    send_can_msg(can_id, data, 8);
    ESP_LOGI(TAG, "Set controller mode: node=%d, ctrl=%d, input=%d", node_id, ctrl_mode, input_mode);
}

void ODrive::set_input_pos(uint8_t node_id, float pos, int16_t vel_ff, int16_t torque_ff)
{
    uint32_t can_id = build_can_id(node_id, CAN_SET_INPUT_POS);
    uint8_t data[8];
    memcpy(&data[0], &pos, 4);
    memcpy(&data[4], &vel_ff, 2);
    memcpy(&data[6], &torque_ff, 2);
    send_can_msg(can_id, data, 8);
}

void ODrive::set_input_vel(uint8_t node_id, float vel, float torque_ff)
{
    uint32_t can_id = build_can_id(node_id, CAN_SET_INPUT_VEL);
    uint8_t data[8];
    memcpy(&data[0], &vel, 4);
    memcpy(&data[4], &torque_ff, 4);
    send_can_msg(can_id, data, 8);
}

void ODrive::set_input_torque(uint8_t node_id, float torque)
{
    uint32_t can_id = build_can_id(node_id, CAN_SET_INPUT_TORQUE);
    send_can_msg(can_id, (uint8_t*)&torque, 4);
}

void ODrive::set_limits(uint8_t node_id, float vel_limit, float current_limit)
{
    uint32_t can_id = build_can_id(node_id, CAN_SET_LIMITS);
    uint8_t data[8];
    memcpy(&data[0], &vel_limit, 4);
    memcpy(&data[4], &current_limit, 4);
    send_can_msg(can_id, data, 8);
    ESP_LOGI(TAG, "Set limits: node=%d, vel=%.2f, current=%.2f", node_id, vel_limit, current_limit);
}

void ODrive::set_pos_gain(uint8_t node_id, float pos_gain)
{
    uint32_t can_id = build_can_id(node_id, CAN_SET_POS_GAIN);
    send_can_msg(can_id, (uint8_t*)&pos_gain, 4);
}

void ODrive::set_vel_gains(uint8_t node_id, float vel_gain, float vel_integrator_gain)
{
    uint32_t can_id = build_can_id(node_id, CAN_SET_VEL_GAINS);
    uint8_t data[8];
    memcpy(&data[0], &vel_gain, 4);
    memcpy(&data[4], &vel_integrator_gain, 4);
    send_can_msg(can_id, data, 8);
}

void ODrive::estop(uint8_t node_id)
{
    uint32_t can_id = build_can_id(node_id, CAN_ESTOP);
    send_can_msg(can_id, nullptr, 0);
    ESP_LOGW(TAG, "E-STOP sent to node %d", node_id);
}

void ODrive::clear_errors(uint8_t node_id)
{
    uint32_t can_id = build_can_id(node_id, CAN_CLEAR_ERRORS);
    send_can_msg(can_id, nullptr, 0);
    ESP_LOGI(TAG, "Clear errors: node=%d", node_id);
}

void ODrive::reboot(uint8_t node_id)
{
    uint32_t can_id = build_can_id(node_id, CAN_REBOOT);
    send_can_msg(can_id, nullptr, 0);
    ESP_LOGI(TAG, "Reboot: node=%d", node_id);
}

void ODrive::request_encoder_est(uint8_t node_id)
{
    uint32_t can_id = build_can_id(node_id, CAN_GET_ENCODER_ESTIMATES);
    send_can_msg(can_id, nullptr, 0);
}

void ODrive::request_iq(uint8_t node_id)
{
    uint32_t can_id = build_can_id(node_id, CAN_GET_IQ);
    send_can_msg(can_id, nullptr, 0);
}

void ODrive::request_bus_voltage_current(uint8_t node_id)
{
    uint32_t can_id = build_can_id(node_id, CAN_GET_BUS_VOLTAGE_CURRENT);
    send_can_msg(can_id, nullptr, 0);
}

void ODrive::request_temperature(uint8_t node_id)
{
    uint32_t can_id = build_can_id(node_id, CAN_GET_TEMPERATURE);
    send_can_msg(can_id, nullptr, 0);
}

void ODrive::set_heartbeat_callback(odrive_heartbeat_cb_t cb, void* ctx)
{
    heartbeat_cb_ = cb;
    heartbeat_ctx_ = ctx;
}

void ODrive::set_encoder_callback(odrive_encoder_cb_t cb, void* ctx)
{
    encoder_cb_ = cb;
    encoder_ctx_ = ctx;
}

void ODrive::set_iq_callback(odrive_iq_cb_t cb, void* ctx)
{
    iq_cb_ = cb;
    iq_ctx_ = ctx;
}

void ODrive::process_msg(const twai_frame_t& msg)
{
    // Extract node ID and command ID from CAN ID
    uint8_t node_id = (msg.header.id >> 5) & 0x3F;
    uint16_t cmd_id = msg.header.id & 0x1F;
    ESP_LOGI(TAG, "RX: %x [%d] %x %x %x %x", \
                     msg.header.id, msg.header.dlc, msg.buffer[0], msg.buffer[1], msg.buffer[2], msg.buffer[3]);
    switch (cmd_id) {
        case CAN_HEARTBEAT:
            parse_heartbeat(node_id, msg.buffer, msg.header.dlc);
            break;
            
        case CAN_GET_ENCODER_ESTIMATES:
            parse_encoder_estimates(node_id, msg.buffer, msg.header.dlc);
            break;
            
        case CAN_GET_IQ:
            parse_iq(node_id, msg.buffer, msg.header.dlc);
            break;
            
        default:
            // Unhandled message type
            break;
    }
}

void ODrive::parse_heartbeat(uint8_t node_id, const uint8_t* data, uint8_t len)
{
    if (len < 8) return;
    
    uint32_t error;
    uint8_t state;
    
    memcpy(&error, &data[0], 4);
    state = data[4];
    // data[5-7] contain procedure result and trajectory done flag
    
    if (heartbeat_cb_) {
        heartbeat_cb_(node_id, error, state, heartbeat_ctx_);
    }
}

void ODrive::parse_encoder_estimates(uint8_t node_id, const uint8_t* data, uint8_t len)
{
    if (len < 8) return;
    
    float pos, vel;
    memcpy(&pos, &data[0], 4);
    memcpy(&vel, &data[4], 4);
    
    if (encoder_cb_) {
        encoder_cb_(node_id, pos, vel, encoder_ctx_);
    }
}

void ODrive::parse_iq(uint8_t node_id, const uint8_t* data, uint8_t len)
{
    if (len < 8) return;
    
    float iq_setpoint, iq_measured;
    memcpy(&iq_setpoint, &data[0], 4);
    memcpy(&iq_measured, &data[4], 4);
    
    if (iq_cb_) {
        iq_cb_(node_id, iq_setpoint, iq_measured, iq_ctx_);
    }
}
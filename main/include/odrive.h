#ifndef ODRIVE_CAN_H
#define ODRIVE_CAN_H

#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <stdint.h>

// ODrive CAN Protocol Commands (CMD_ID portion of arbitration ID)
static const uint32_t CONTROL_MODE_VOLTAGE_CONTROL = 0x0;
  static const uint32_t CONTROL_MODE_TORQUE_CONTROL = 0x1;
  static const uint32_t CONTROL_MODE_VELOCITY_CONTROL = 0x2;
  static const uint32_t CONTROL_MODE_POSITION_CONTROL = 0x3;

  static const uint32_t CAN_GET_VERSION = 0x000;
  static const uint32_t CAN_HEARTBEAT = 0x001;
  static const uint32_t CAN_ESTOP = 0x002;
  static const uint32_t CAN_GET_ERRORS = 0x003;
  static const uint32_t CAN_RXSDO = 0x004;
  static const uint32_t CAN_TXSDO = 0x005;
  static const uint32_t CAN_ADDRESS = 0x006;
  static const uint32_t CAN_SET_AXIS_STATE = 0x007;
  static const uint32_t CAN_GET_ENCODER_ESTIMATES = 0x009;
  static const uint32_t CAN_SET_CONTROLLER_MODE = 0x00b;
  static const uint32_t CAN_SET_INPUT_POS = 0x00c;
  static const uint32_t CAN_SET_INPUT_VEL = 0x00d;
  static const uint32_t CAN_SET_INPUT_TORQUE = 0x00e;
  static const uint32_t CAN_SET_LIMITS = 0x00f;
  static const uint32_t CAN_SET_TRAJ_VEL_LIMIT = 0x011;
  static const uint32_t CAN_SET_TRAJ_ACCEL_LIMITS = 0x012;
  static const uint32_t CAN_SET_TRAJ_INERTIA = 0x013;
  static const uint32_t CAN_GET_IQ = 0x014;
  static const uint32_t CAN_GET_TEMPERATURE = 0x015;
  static const uint32_t CAN_REBOOT = 0x016;
  static const uint32_t CAN_GET_BUS_VOLTAGE_CURRENT = 0x017;
  static const uint32_t CAN_CLEAR_ERRORS = 0x018;
  static const uint32_t CAN_SET_ABSOLUTE_POSITION = 0x019;
  static const uint32_t CAN_SET_POS_GAIN = 0x01a;
  static const uint32_t CAN_SET_VEL_GAINS = 0x01b;
  static const uint32_t CAN_GET_TORQUES = 0x01c;
  static const uint32_t CAN_GET_POWERS = 0x01d;
  static const uint32_t CAN_ENTER_DFU_MODE = 0x01f;

  static const uint8_t INIT_SUCCESS = 0;
  static const uint8_t INIT_CAN_ERROR = 1;

  static const uint8_t CMD_SUCCESS = 0;
  static const uint8_t CMD_ERROR_INVALID_AXIS = 1;
  static const uint8_t CMD_ERROR_INVALID_COMMAND = 2;
  static const uint8_t CMD_ERROR_WRITE_FAILED = 3;

// ODrive Axis States
typedef enum {
    AXIS_STATE_UNDEFINED = 0,
    AXIS_STATE_IDLE = 1,
    AXIS_STATE_STARTUP_SEQUENCE = 2,
    AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3,
    AXIS_STATE_MOTOR_CALIBRATION = 4,
    AXIS_STATE_ENCODER_INDEX_SEARCH = 6,
    AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7,
    AXIS_STATE_CLOSED_LOOP_CONTROL = 8,
    AXIS_STATE_LOCKIN_SPIN = 9,
    AXIS_STATE_ENCODER_DIR_FIND = 10,
    AXIS_STATE_HOMING = 11,
    AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION = 12,
    AXIS_STATE_ENCODER_HALL_PHASE_CALIBRATION = 13,
    AXIS_STATE_ANTICOGGING_CALIBRATION = 14,
} odrive_axis_state_t;

// ODrive Control Modes
typedef enum {
    CTRL_MODE_VOLTAGE_CONTROL = 0,
    CTRL_MODE_TORQUE_CONTROL = 1,
    CTRL_MODE_VELOCITY_CONTROL = 2,
    CTRL_MODE_POSITION_CONTROL = 3,
} odrive_control_mode_t;

// ODrive Input Modes
typedef enum {
    INPUT_MODE_INACTIVE = 0,
    INPUT_MODE_PASSTHROUGH = 1,
    INPUT_MODE_VEL_RAMP = 2,
    INPUT_MODE_POS_FILTER = 3,
    INPUT_MODE_MIX_CHANNELS = 4,
    INPUT_MODE_TRAP_TRAJ = 5,
    INPUT_MODE_TORQUE_RAMP = 6,
    INPUT_MODE_MIRROR = 7,
    INPUT_MODE_TUNING = 8,
} odrive_input_mode_t;
#pragma pack(push, 1)
struct CanMessage {
    uint32_t id;
    uint8_t len;
    uint8_t data[8];
};
#pragma pack(pop)

// The FreeRTOS Queue Handle


// Callback types
typedef void (*odrive_heartbeat_cb_t)(uint8_t node_id, uint32_t error, uint8_t state, void* ctx);
typedef void (*odrive_encoder_cb_t)(uint8_t node_id, float pos, float vel, void* ctx);
typedef void (*odrive_iq_cb_t)(uint8_t node_id, float iq_setpoint, float iq_measured, void* ctx);

class ODrive {
public:
    ODrive();
    ~ODrive();

    // Initialize CAN bus
    bool init(gpio_num_t tx_pin, gpio_num_t rx_pin, uint32_t bitrate = 500000);
    
    // Start/stop CAN driver
    bool start();
    void stop();

    



    // Command functions
    void set_axis_state(uint8_t node_id, odrive_axis_state_t state);
    void set_controller_mode(uint8_t node_id, odrive_control_mode_t ctrl_mode, odrive_input_mode_t input_mode);
    void set_input_mode(uint8_t node_id, float pos, int16_t vel_ff = 0, int16_t torque_ff = 0);
    void set_input_vel(uint8_t node_id, float vel, float torque_ff = 0.0f);
    void set_input_torque(uint8_t node_id, float torque);
    void set_input_pos(uint8_t node_id, float pos, int16_t vel_ff, int16_t torque_ff);
    void set_limits(uint8_t node_id, float vel_limit, float current_limit);
    void set_pos_gain(uint8_t node_id, float pos_gain);
    void set_vel_gains(uint8_t node_id, float vel_gain, float vel_integrator_gain);
    void estop(uint8_t node_id);
    void clear_errors(uint8_t node_id);
    void reboot(uint8_t node_id);

    // Request functions
    void request_encoder_est(uint8_t node_id);
    void request_iq(uint8_t node_id);
    void request_bus_voltage_current(uint8_t node_id);
    void request_temperature(uint8_t node_id);

    // Callback registration
    void set_heartbeat_callback(odrive_heartbeat_cb_t cb, void* ctx);
    void set_encoder_callback(odrive_encoder_cb_t cb, void* ctx);
    void set_iq_callback(odrive_iq_cb_t cb, void* ctx);

private:
    static QueueHandle_t can_tx_queue;
    struct RxFrameBuffer {
        twai_frame_t frame;
        uint8_t data[8];
    };
    // CAN message construction
    uint32_t build_can_id(uint8_t node_id, uint16_t cmd_id);
    void send_can_msg(uint32_t can_id, const uint8_t* data, uint8_t len, bool remote = false);
    
    static bool IRAM_ATTR on_rx_done_ISR(twai_node_handle_t handle, const twai_rx_done_event_data_t* edata, void* user_ctx);
    static bool IRAM_ATTR on_error_ISR(twai_node_handle_t handle, const twai_error_event_data_t* edata, void* user_ctx);
    static bool IRAM_ATTR on_state_change_ISR(twai_node_handle_t handle, const twai_state_change_event_data_t* edata, void* user_ctx);


    // RX task
    static void can_tx_task(void* pvParameters);
    static void rx_task_entry(void* arg);
    void rx_task();
    void process_msg(const twai_frame_t& msg);

    // Helper functions for parsing
    void parse_heartbeat(uint8_t node_id, const uint8_t* data, uint8_t len);
    void parse_encoder_estimates(uint8_t node_id, const uint8_t* data, uint8_t len);
    void parse_iq(uint8_t node_id, const uint8_t* data, uint8_t len);

    // Configuration
    gpio_num_t tx_pin_;
    gpio_num_t rx_pin_;
    uint32_t bitrate_;
    int rx_buffer_depth_;

    twai_node_handle_t node_handle_;
    // Task handle

    RxFrameBuffer* rx_pool_;
    volatile int write_idx_;
    volatile int read_idx_;

    // Semaphores for flow control
    SemaphoreHandle_t free_pool_sem_;
    SemaphoreHandle_t rx_ready_sem_;

    // Task handle
    TaskHandle_t rx_task_handle_;
    volatile bool running_;

    // Callbacks
    odrive_heartbeat_cb_t heartbeat_cb_;
    void* heartbeat_ctx_;
    odrive_encoder_cb_t encoder_cb_;
    void* encoder_ctx_;
    odrive_iq_cb_t iq_cb_;
    void* iq_ctx_;
};

#endif 
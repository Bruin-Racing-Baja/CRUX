#include "sensors/imu_sensor.h"
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

// SH-2 channels
#define CH_COMMAND          0
#define CH_EXECUTABLE       1
#define CH_CONTROL          2
#define CH_INPUT_REPORTS    3
#define CH_WAKE_REPORTS     4
#define CH_GYRO_INTEGRATED  5

// Sensor report IDs (SH-2 reference manual section 6.5)
#define REPORT_ACCELEROMETER     0x01
#define REPORT_GYROSCOPE         0x02
#define REPORT_ROTATION_VECTOR   0x05

// Q-point scaling (fixed-point exponents from datasheet)
#define ACCEL_Q_POINT     8
#define GYRO_Q_POINT      9
#define ROTATION_Q_POINT  14

// SH-2 commands
#define SET_FEATURE_COMMAND  0xFD

IMUSensor::IMUSensor(uint8_t mosi_pin_, uint8_t miso_pin_, uint8_t sck_pin_,
                     uint8_t cs_pin_, uint8_t int_pin_, uint8_t rst_pin_)
    : mosi_pin(mosi_pin_), miso_pin(miso_pin_), sck_pin(sck_pin_),
      cs_pin(cs_pin_), int_pin(int_pin_), rst_pin(rst_pin_),
      yaw_deg(0), pitch_deg(0), roll_deg(0),
      accel_x(0), accel_y(0), accel_z(0),
      gyro_x(0), gyro_y(0), gyro_z(0),
      qw(1), qi(0), qj(0), qk(0),
      ready(false)
{
    memset(seq_nums, 0, sizeof(seq_nums));
    // constructor does NOTHING that touches FreeRTOS or hardware
}

void IMUSensor::init() {
    pinMode(cs_pin,  PinMode::OUTPUT_ONLY);
    pinMode(rst_pin, PinMode::OUTPUT_ONLY);
    pinMode(int_pin, PinMode::INPUT_ONLY);
    cs_high();

    spi_bus_config_t bus_config = {
        .mosi_io_num     = mosi_pin,
        .miso_io_num     = miso_pin,
        .sclk_io_num     = sck_pin,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = IMU_PACKET_SIZE,
    };
    spi_bus_initialize(IMU_SPI_HOST, &bus_config, SPI_DMA_CH_AUTO);

    spi_device_interface_config_t dev_config = {
        .mode           = 3,
        .clock_speed_hz = IMU_SPI_FREQ_HZ,
        .spics_io_num   = -1,
        .queue_size     = 1,
    };
    spi_bus_add_device(IMU_SPI_HOST, &dev_config, &spi_handle);

    hardware_reset();

    enable_report(REPORT_ROTATION_VECTOR, 10000);
    enable_report(REPORT_ACCELEROMETER,   10000);
    enable_report(REPORT_GYROSCOPE,       10000);

    ready = true;
}

// pulse RST low to fully reset; on boot the sensor sends advertisement packets
// that we need to drain before normal traffic
void IMUSensor::hardware_reset() {
    gpio_set_level((gpio_num_t)rst_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level((gpio_num_t)rst_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(300));   // datasheet says ~200 ms boot

    // drain startup advertisement / unsolicited init packets
    for (int i = 0; i < 8; i++) {
        if (!receive_packet()) break;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// BNO085 pulls INT low when it has data or is ready to accept a write
bool IMUSensor::wait_for_int(uint32_t timeout_ms) {
    uint32_t start = esp_timer_get_time() / 1000;
    while ((esp_timer_get_time() / 1000) - start < timeout_ms) {
        if (gpio_get_level((gpio_num_t)int_pin) == 0) return true;
        vTaskDelay(1);
    }
    return false;
}

// SH-2 packet header: [len_lsb][len_msb][channel][seq_num], then payload
bool IMUSensor::send_packet(uint8_t channel, const uint8_t* data, uint16_t length) {
    uint16_t total_len = length + 4;

    tx_buffer[0] = total_len & 0xFF;
    tx_buffer[1] = (total_len >> 8) & 0x7F;   // top bit reserved
    tx_buffer[2] = channel;
    tx_buffer[3] = seq_nums[channel]++;
    memcpy(&tx_buffer[4], data, length);

    if (!wait_for_int(100)) return false;

    spi_transaction_t trans = {};
    trans.length    = total_len * 8;
    trans.tx_buffer = tx_buffer;
    trans.rx_buffer = rx_buffer;

    cs_low();
    esp_err_t ret = spi_device_polling_transmit(spi_handle, &trans);
    cs_high();

    return ret == ESP_OK;
}

// reads a single SH-2 packet; reads header first to learn length, then body
bool IMUSensor::receive_packet() {
    if (gpio_get_level((gpio_num_t)int_pin) != 0) return false;

    // read 4 byte header
    spi_transaction_t header = {};
    header.length    = 4 * 8;
    header.tx_buffer = NULL;
    header.rx_buffer = rx_buffer;

    cs_low();
    if (spi_device_polling_transmit(spi_handle, &header) != ESP_OK) {
        cs_high();
        return false;
    }

    uint16_t packet_len = ((rx_buffer[1] & 0x7F) << 8) | rx_buffer[0];
    uint8_t  channel    = rx_buffer[2];

    // packet_len of 0 = nothing to send; otherwise must fit in our buffer
    if (packet_len == 0 || packet_len > IMU_PACKET_SIZE) {
        cs_high();
        return false;
    }

    // read remaining payload
    if (packet_len > 4) {
        spi_transaction_t body = {};
        body.length    = (packet_len - 4) * 8;
        body.tx_buffer = NULL;
        body.rx_buffer = &rx_buffer[4];
        spi_device_polling_transmit(spi_handle, &body);
    }
    cs_high();

    if (channel == CH_INPUT_REPORTS) {
        parse_input_report();
    }
    return true;
}

// input reports start with a 5 byte timestamp/cargo header before the actual report
void IMUSensor::parse_input_report() {
    uint8_t* payload = &rx_buffer[4 + 5];
    uint8_t  report_id = payload[0];

    auto read_int16 = [](const uint8_t* p) -> int16_t {
        return (int16_t)(p[0] | (p[1] << 8));
    };

    if (report_id == REPORT_ROTATION_VECTOR) {
        // bytes 0..3 are report id / status / delay; quaternion starts at offset 4
        float scale = 1.0f / (1 << ROTATION_Q_POINT);
        qi = read_int16(&payload[4])  * scale;
        qj = read_int16(&payload[6])  * scale;
        qk = read_int16(&payload[8])  * scale;
        qw = read_int16(&payload[10]) * scale;
        quaternion_to_euler();
    }
    else if (report_id == REPORT_ACCELEROMETER) {
        float scale = 1.0f / (1 << ACCEL_Q_POINT);
        accel_x = read_int16(&payload[4]) * scale;
        accel_y = read_int16(&payload[6]) * scale;
        accel_z = read_int16(&payload[8]) * scale;
    }
    else if (report_id == REPORT_GYROSCOPE) {
        float scale = 1.0f / (1 << GYRO_Q_POINT);
        gyro_x = read_int16(&payload[4]) * scale;
        gyro_y = read_int16(&payload[6]) * scale;
        gyro_z = read_int16(&payload[8]) * scale;
    }
}

// converts unit quaternion to yaw/pitch/roll in degrees (ZYX intrinsic)
void IMUSensor::quaternion_to_euler() {
    // roll (x-axis rotation)
    float sinr_cosp = 2.0f * (qw * qi + qj * qk);
    float cosr_cosp = 1.0f - 2.0f * (qi * qi + qj * qj);
    roll_deg = atan2f(sinr_cosp, cosr_cosp) * 180.0f / (float)M_PI;

    // pitch (y-axis rotation); clamp at +/- 90 to avoid gimbal lock NaNs
    float sinp = 2.0f * (qw * qj - qk * qi);
    if (fabsf(sinp) >= 1.0f)
        pitch_deg = copysignf((float)M_PI / 2.0f, sinp) * 180.0f / (float)M_PI;
    else
        pitch_deg = asinf(sinp) * 180.0f / (float)M_PI;

    // yaw (z-axis rotation)
    float siny_cosp = 2.0f * (qw * qk + qi * qj);
    float cosy_cosp = 1.0f - 2.0f * (qj * qj + qk * qk);
    yaw_deg = atan2f(siny_cosp, cosy_cosp) * 180.0f / (float)M_PI;
}

// SH-2 "Set Feature Command" tells the sensor to start producing a given report
// at the requested interval (microseconds, little-endian)
void IMUSensor::enable_report(uint8_t report_id, uint32_t interval_us) {
    uint8_t cmd[17] = {0};
    cmd[0] = SET_FEATURE_COMMAND;
    cmd[1] = report_id;
    cmd[5] = interval_us         & 0xFF;
    cmd[6] = (interval_us >> 8)  & 0xFF;
    cmd[7] = (interval_us >> 16) & 0xFF;
    cmd[8] = (interval_us >> 24) & 0xFF;

    send_packet(CH_CONTROL, cmd, sizeof(cmd));
    vTaskDelay(pdMS_TO_TICKS(5));
}

// drain everything the sensor has queued up since last call
void IMUSensor::update() {
    while (gpio_get_level((gpio_num_t)int_pin) == 0) {
        if (!receive_packet()) break;
    }

    static int count = 0;
    if (++count % 50 == 0) {
        printf("yaw=%.1f pitch=%.1f roll=%.1f ax=%.2f\n",
               yaw_deg, pitch_deg, roll_deg, accel_x);
    }
}
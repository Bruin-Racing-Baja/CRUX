#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <stdint.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "gpio_wrapper.h"

#define IMU_SPI_HOST       SPI2_HOST
#define IMU_SPI_FREQ_HZ    3000000   // BNO085 max is 3 MHz
#define IMU_PACKET_SIZE    256

class IMUSensor {
public:
    IMUSensor(uint8_t mosi_pin_, uint8_t miso_pin_, uint8_t sck_pin_,
              uint8_t cs_pin_, uint8_t int_pin_, uint8_t rst_pin_);
    void update();
    void init();

    float yaw()   const { return yaw_deg; }
    float pitch() const { return pitch_deg; }
    float roll()  const { return roll_deg; }

    float ax() const { return accel_x; }
    float ay() const { return accel_y; }
    float az() const { return accel_z; }

    float gx() const { return gyro_x; }
    float gy() const { return gyro_y; }
    float gz() const { return gyro_z; }

    bool is_ready() const { return ready; }

private:
    const uint8_t mosi_pin, miso_pin, sck_pin;
    const uint8_t cs_pin, int_pin, rst_pin;

    spi_device_handle_t spi_handle;

    float yaw_deg, pitch_deg, roll_deg;
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float qw, qi, qj, qk;          // unit quaternion from rotation vector

    uint8_t seq_nums[6];           // SH-2 has 6 channels, each tracks its own seq num
    uint8_t tx_buffer[IMU_PACKET_SIZE];
    uint8_t rx_buffer[IMU_PACKET_SIZE];

    bool ready;

    void cs_low()  { gpio_set_level((gpio_num_t)cs_pin, 0); }
    void cs_high() { gpio_set_level((gpio_num_t)cs_pin, 1); }

    void hardware_reset();
    bool wait_for_int(uint32_t timeout_ms);
    bool send_packet(uint8_t channel, const uint8_t* data, uint16_t length);
    bool receive_packet();
    void parse_input_report();
    void enable_report(uint8_t report_id, uint32_t interval_us);
    void quaternion_to_euler();
};

#endif
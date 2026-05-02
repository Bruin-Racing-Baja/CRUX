#ifndef GPS_SENSOR_H
#define GPS_SENSOR_H

#include <stdint.h>
#include "driver/uart.h"
#include "gpio_wrapper.h"

#define GPS_BAUD_RATE 9600
#define GPS_BUFFER_SIZE 128

class GPS {
public:
    GPS(uint8_t tx_pin_, uint8_t rx_pin_, uart_port_t uart_port_ = UART_NUM_1, uint32_t baud_rate_ = GPS_BAUD_RATE);
    void update();

    float get_latitude() const { return latitude; }
    float get_longitude() const { return longitude; }
    float get_speed_mps() const { return speed_mps; }
    float get_heading_deg() const { return heading_deg; }
    bool get_has_fix() const { return has_fix; }

private:
    const uart_port_t uart_port;
    const uint8_t tx_pin;
    const uint8_t rx_pin;

    float latitude;
    float longitude;
    float speed_mps;
    float heading_deg;
    bool has_fix;

    char buffer[GPS_BUFFER_SIZE];
    uint8_t buffer_index;

    bool read_sentence();
    void parse_rmc(const char* sentence);
    float parse_coordinate(const char* raw, const char* direction);
};

#endif
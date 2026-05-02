#include "sensors/gps_sensor.h"
#include <string.h>
#include <stdlib.h>

GPS::GPS(uint8_t tx_pin_, uint8_t rx_pin_, uart_port_t uart_port_, uint32_t baud_rate_)
        : uart_port(uart_port_), tx_pin(tx_pin_), rx_pin(rx_pin_), latitude(0.0f), longitude(0.0f), speed_mps(0.0f), heading_deg(0.0f), has_fix(false), buffer_index(0)
{
    pinMode(rx_pin, PinMode::INPUT_ONLY);
    pinMode(tx_pin, PinMode::OUTPUT_ONLY);

    // create config
    uart_config_t uart_config = {
        .baud_rate = GPS_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT, 
    };

    // apply config
    uart_param_config(uart_port, &uart_config);
    // assign pins
    uart_set_pin(uart_port, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // install driver with receiver buffer; want driver's size to be greater than buffer's size
    uart_driver_install(uart_port, GPS_BUFFER_SIZE * 2, 0, 0, NULL, 0); 
}

// reads and records the sentence in buffer
bool GPS::read_sentence() {
    uint8_t c;
    // read 1 byte at a time; 0 represents do not wait
    while (uart_read_bytes(uart_port, &c, 1, 0) == 1) {
        //printf("%c", (char)c);
        if (c == '\n') {
            buffer[buffer_index] = '\0';
            buffer_index = 0;
            return true;
        }

        if (buffer_index < GPS_BUFFER_SIZE - 1) {
            buffer[buffer_index++] = (char)c;
        }
    }
    return false;
}

// coordinates are in DDMM.MMMMM; returns coordinate in degrees
float GPS::parse_coordinate(const char* raw, const char* direction) {
    float raw_val = atof(raw);
    // first two digits of coords are degrees
    int degrees = (int)(raw_val / 100);
    
    float minutes = raw_val - degrees * 100;
    float decimal_degrees = degrees + minutes / 60.0f;

    if (direction[0] == 'S' || direction[0] == 'W') {
        decimal_degrees *= -1;
    }
    return decimal_degrees;
}

void GPS::parse_rmc(const char* sentence) {
    char sentence_copy[GPS_BUFFER_SIZE];
    strncpy(sentence_copy, sentence, GPS_BUFFER_SIZE);

    char* field = strtok(sentence_copy, ","); // $GPRMC
    field = strtok(NULL, ","); // time
    field = strtok(NULL, ","); // tells use whether gps is fixed: A = active, V = void

    has_fix = field[0] == 'A';
    if (!has_fix) return;

    char* lat_raw = strtok(NULL, ",");
    char* lat_dir = strtok(NULL, ",");
    char* lon_raw = strtok(NULL, ",");
    char* lon_dir = strtok(NULL, ",");
    char* speed = strtok(NULL, ",");
    char* heading = strtok(NULL, ",");

    latitude = parse_coordinate(lat_raw, lat_dir);
    longitude = parse_coordinate(lon_raw, lon_dir);
    speed_mps = atof(speed) * 0.514444f;
    heading_deg = atof(heading);
}

// updates buffer; we only care about the $GPRMC, since it has the vals we're looking for
void GPS::update() {
    while (read_sentence()) {
        if (strncmp(buffer, "$GPRMC", 6) == 0)
            parse_rmc(buffer);
    }

    static int count = 0;
    if (++count % 100 == 0) {
        printf("GPS: fix=%d lat=%.6f lon=%.6f speed=%.2f\n",
               has_fix, latitude, longitude, speed_mps);
    }
}
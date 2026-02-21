#include "input_output/shift_register.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

ShiftRegister::ShiftRegister(uint32_t ser_in_pin_, 
                             uint32_t shift_reg_clk_pin_, 
                             uint32_t reg_clk_pin_)
    :SER_IN_PIN(ser_in_pin_), 
    SHIFT_REG_CLK_PIN(shift_reg_clk_pin_), 
    REG_CLK_PIN(reg_clk_pin_), 
    data(0) 
{
    /* Need this to INPUT_ONLY b/c they are the 1.8V ones */
    pinMode(47, PinMode::INPUT_ONLY);
    pinMode(48, PinMode::INPUT_ONLY);

    pinMode(SER_IN_PIN, PinMode::OUTPUT_ONLY);
    pinMode(SHIFT_REG_CLK_PIN, PinMode::OUTPUT_ONLY);
    pinMode(REG_CLK_PIN, PinMode::OUTPUT_ONLY);
}

void ShiftRegister::write_byte(uint8_t byte)
{
    digitalWrite(REG_CLK_PIN, LOW);

    for (int i = 7; i >= 0; i--) {
        digitalWrite(SER_IN_PIN, (byte >> i) & 0x01);
        digitalWrite(SHIFT_REG_CLK_PIN, HIGH);
        digitalWrite(SHIFT_REG_CLK_PIN, LOW);
    }

    digitalWrite(REG_CLK_PIN, HIGH);

    data = byte;
}

bool ShiftRegister::write_led(uint8_t led_num, bool value) {
    if (led_num < 0 || led_num > 7)
        return false; 

    write_byte((data & ~(1u<<led_num)) | (value<<led_num)); // replaces n-th bit with value and writes to register
    return true;
}
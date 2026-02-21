#ifndef SHIFT_REGISTER_H
#define SHIFT_REGISTER_H

#include "gpio_wrapper.h"

class ShiftRegister {
public:
    ShiftRegister(uint32_t ser_in_pin_, 
                  uint32_t shift_reg_clk_pin_, 
                  uint32_t reg_clk_pin_);
    
    bool write_led(uint8_t led_num, bool value);

    uint8_t get_current_state() const {return data;} 
    
private:
    void write_byte(uint8_t byte);

    const uint32_t SER_IN_PIN;             // serial input
    const uint32_t SHIFT_REG_CLK_PIN;      // shift register clock
    const uint32_t REG_CLK_PIN;            // register clock (activates latch)
    
    uint8_t data;                    // current data on shift register (default 0) // essentially current state
};

#endif // SHIFT_REGISTER_H
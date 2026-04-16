#ifndef LED_H
#define LED_H

#include <vector>
#include "gpio_wrapper.h"
#include "freertos/FreeRTOS.h"

using std::vector;

class LED {
public:
    LED (vector<int> pins) : pins(pins), led_state(pins.size(), false) 
    {
        for (int pin : pins) {
            pinMode(pin, PinMode::OUTPUT_ONLY);
            
            /* Blink LED on Startup */
            digitalWrite(pin, HIGH); 
            vTaskDelay(pdMS_TO_TICKS(500));
            digitalWrite(pin, LOW); 
        }
    }

    void turn_on_all_leds(){
        for (int pin : pins){
            digitalWrite(pin, HIGH);
        }
    }
    void turn_off_all_leds(){
        for (int pin : pins){
            digitalWrite(pin, LOW);
        }
    }

    void turn_on_led(int led_pin){
        digitalWrite(led_pin, HIGH);
    }
    void turn_off_led(int led_pin){
        digitalWrite(led_pin, LOW);
    }

private:
    vector<int> pins;

    vector<bool> led_state;
};

#endif // LED_H
#include "gpio_wrapper.h"
extern "C" {
    #include "driver/gpio.h"
}
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"

bool adc_initialized = false;
adc_oneshot_unit_handle_t adc_handle;

void pinMode(int pin, PinMode mode) {
    gpio_config_t io_conf = {};

    // Disable interrupts by default
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // Set bit mask for the specific pin
    io_conf.pin_bit_mask = (1ULL << pin);

    switch (mode) {
        case PinMode::OUTPUT_ONLY:
            io_conf.mode = GPIO_MODE_OUTPUT;
            io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
            io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
            break;

        case PinMode::INPUT_ONLY:
            io_conf.mode = GPIO_MODE_INPUT;
            io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
            io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
            break;

        case PinMode::INPUT_PULLUP:
            io_conf.mode = GPIO_MODE_INPUT;
            io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
            io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
            break;

        case PinMode::INPUT_PULLDOWN:
            io_conf.mode = GPIO_MODE_INPUT;
            io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
            io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
            break;

        case PinMode::INPUT_OUTPUT_OD:
            io_conf.mode = GPIO_MODE_INPUT_OUTPUT_OD;
            io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
            io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
            break;
    }

    gpio_config(&io_conf);
}

//Make sure to add IRAM_ATTR before any isr handler functions
void attachInterrupt(int pin, IsrHandler handler, InterruptMode mode, void* arg) {
    static bool isr_service_installed = false;
    
    // Installs the global GPIO ISR service 
    // ESP_INTR_FLAG_IRAM ensures interrupts work even when flash is disabled
    if (!isr_service_installed) {
        esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
        if (err == ESP_OK || err == ESP_ERR_INVALID_STATE) {
            isr_service_installed = true;
        }
    }

    gpio_int_type_t intr_type = GPIO_INTR_DISABLE;
    switch (mode) {
        case InterruptMode::RISING_EDGE:  intr_type = GPIO_INTR_POSEDGE; break;
        case InterruptMode::FALLING_EDGE: intr_type = GPIO_INTR_NEGEDGE; break;
        case InterruptMode::ANY_CHANGE:   intr_type = GPIO_INTR_ANYEDGE; break;
        case InterruptMode::LOW_LEVEL:    intr_type = GPIO_INTR_LOW_LEVEL; break;
        case InterruptMode::HIGH_LEVEL:   intr_type = GPIO_INTR_HIGH_LEVEL; break;
    }
    gpio_set_intr_type(static_cast<gpio_num_t>(pin), intr_type);

    gpio_isr_handler_add(static_cast<gpio_num_t>(pin), handler, arg);
}

void digitalWrite(int pin, int level) {
    gpio_set_level(static_cast<gpio_num_t>(pin), level);
}

void digitalWrite(int pin, bool level) {
    gpio_num_t gpio_num = static_cast<gpio_num_t>(pin);
    gpio_set_level(gpio_num, level ? 1 : 0);
}

int digitalRead(int pin) {
    return gpio_get_level(static_cast<gpio_num_t>(pin));
}

int analogRead(int pin) {
    // 1. Initialize ADC Unit (only once)
    if (!adc_initialized) {
        adc_oneshot_unit_init_cfg_t init_config = {};
        init_config.unit_id = ADC_UNIT_1; // Safest unit (ADC2 conflicts with WiFi)
        init_config.ulp_mode = ADC_ULP_MODE_DISABLE;
        
        ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));
        adc_initialized = true;
    }

    // 2. Get the ADC channel for this specific GPIO pin
    adc_channel_t channel;
    adc_unit_t unit; 
    
    // Note: This function requires valid ADC1 pins (GPIO 32-39 usually)
    if (adc_oneshot_io_to_channel(pin, &unit, &channel) != ESP_OK) {
        return -1; // Invalid pin for ADC1
    }

    // 3. Configure the channel (Atten 12dB allows reading up to ~3.3V)
    adc_oneshot_chan_cfg_t config = {};
    config.bitwidth = ADC_BITWIDTH_DEFAULT;
    config.atten = ADC_ATTEN_DB_12; 
    
    adc_oneshot_config_channel(adc_handle, channel, &config);

    // 4. Read
    int raw_val = 0;
    adc_oneshot_read(adc_handle, channel, &raw_val);
    return raw_val;
}
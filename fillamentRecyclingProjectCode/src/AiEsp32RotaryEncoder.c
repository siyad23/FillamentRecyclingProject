/**
 * AiEsp32RotaryEncoder - ESP-IDF Port
 * 
 * This is an ESP-IDF compatible port of the AiEsp32RotaryEncoder library
 * by Igor Antolic (igorantolic/ai-esp32-rotary-encoder)
 * 
 * Original library: https://github.com/igorantolic/ai-esp32-rotary-encoder
 * License: MIT
 */

#include "AiEsp32RotaryEncoder.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "RotaryEncoder";

// Encoder lookup table for quadrature decoding - Full step decoding for reliability
// This table provides better noise immunity
static const int8_t enc_states[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

// Alternative half-step lookup table for more sensitive detection
static const int8_t enc_states_half[] = {
    0,  1, -1,  0,
   -1,  0,  0,  1,
    1,  0,  0, -1,
    0, -1,  1,  0
};

// Global pointer to encoder for ISR (supports single encoder instance)
static AiEsp32RotaryEncoder *g_encoder_instance = NULL;

// Get current time in milliseconds
static inline uint32_t millis(void) {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

// ISR handler wrapper
static void IRAM_ATTR gpio_isr_handler(void *arg) {
    AiEsp32RotaryEncoder *encoder = (AiEsp32RotaryEncoder *)arg;
    if (encoder != NULL) {
        AiEsp32RotaryEncoder_readEncoder_ISR(encoder);
    }
}

void AiEsp32RotaryEncoder_init(AiEsp32RotaryEncoder *encoder,
                                gpio_num_t encoder_a_pin,
                                gpio_num_t encoder_b_pin,
                                gpio_num_t encoder_button_pin,
                                gpio_num_t encoder_vcc_pin,
                                uint8_t encoder_steps) {
    memset(encoder, 0, sizeof(AiEsp32RotaryEncoder));
    
    encoder->encoder_a_pin = encoder_a_pin;
    encoder->encoder_b_pin = encoder_b_pin;
    encoder->encoder_button_pin = encoder_button_pin;
    encoder->encoder_vcc_pin = encoder_vcc_pin;
    encoder->encoder_steps = encoder_steps;
    
    encoder->_minEncoderValue = -2147483648LL;
    encoder->_maxEncoderValue = 2147483647LL;
    encoder->circleValues = false;
    encoder->_acceleration = 0;
    encoder->_pauseLength = 200;
    encoder->_fastRotationLimit = 60;
    
    encoder->old_AB = 0;
    encoder->encoder_position = 0;
    encoder->last_read_encoder_position = 0;
    
    encoder->_buttonState = BUT_UP;
    encoder->_isButtonPulldown = false;
    encoder->_buttonDisabled = false;
    
    g_encoder_instance = encoder;
}

void AiEsp32RotaryEncoder_initButtonPulldown(AiEsp32RotaryEncoder *encoder,
                                              gpio_num_t encoder_a_pin,
                                              gpio_num_t encoder_b_pin,
                                              gpio_num_t encoder_button_pin,
                                              gpio_num_t encoder_vcc_pin,
                                              uint8_t encoder_steps) {
    AiEsp32RotaryEncoder_init(encoder, encoder_a_pin, encoder_b_pin, 
                               encoder_button_pin, encoder_vcc_pin, encoder_steps);
    encoder->_isButtonPulldown = true;
}

void AiEsp32RotaryEncoder_begin(AiEsp32RotaryEncoder *encoder) {
    // Configure encoder pins with internal pull-ups
    gpio_config_t io_conf_encoder = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << encoder->encoder_a_pin) | (1ULL << encoder->encoder_b_pin),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io_conf_encoder);
    
    // Configure button pin
    if (encoder->encoder_button_pin >= 0) {
        gpio_config_t io_conf_button = {
            .intr_type = GPIO_INTR_DISABLE,
            .mode = GPIO_MODE_INPUT,
            .pin_bit_mask = (1ULL << encoder->encoder_button_pin),
            .pull_down_en = encoder->_isButtonPulldown ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
            .pull_up_en = encoder->_isButtonPulldown ? GPIO_PULLUP_DISABLE : GPIO_PULLUP_ENABLE,
        };
        gpio_config(&io_conf_button);
    }
    
    // Configure VCC pin if used
    if (encoder->encoder_vcc_pin >= 0) {
        gpio_config_t io_conf_vcc = {
            .intr_type = GPIO_INTR_DISABLE,
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = (1ULL << encoder->encoder_vcc_pin),
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .pull_up_en = GPIO_PULLUP_DISABLE,
        };
        gpio_config(&io_conf_vcc);
        gpio_set_level(encoder->encoder_vcc_pin, 1);
    }
    
    // Read initial state
    encoder->old_AB = 0;
    if (gpio_get_level(encoder->encoder_a_pin)) encoder->old_AB |= (1 << 0);
    if (gpio_get_level(encoder->encoder_b_pin)) encoder->old_AB |= (1 << 1);
    
    ESP_LOGI(TAG, "Rotary encoder initialized - A:%d B:%d BTN:%d", 
             encoder->encoder_a_pin, encoder->encoder_b_pin, encoder->encoder_button_pin);
}

void AiEsp32RotaryEncoder_setup(AiEsp32RotaryEncoder *encoder, void (*isr_callback)(void)) {
    (void)isr_callback; // Not used in this implementation
    
    // Install GPIO ISR service if not already installed
    gpio_install_isr_service(0);
    
    // Add ISR handlers for encoder pins
    gpio_isr_handler_add(encoder->encoder_a_pin, gpio_isr_handler, encoder);
    gpio_isr_handler_add(encoder->encoder_b_pin, gpio_isr_handler, encoder);
    
    ESP_LOGI(TAG, "Rotary encoder ISR setup complete");
}

void IRAM_ATTR AiEsp32RotaryEncoder_readEncoder_ISR(AiEsp32RotaryEncoder *encoder) {
    // Read current state
    int8_t ENC_PORT = 0;
    if (gpio_get_level(encoder->encoder_a_pin)) ENC_PORT |= (1 << 0);
    if (gpio_get_level(encoder->encoder_b_pin)) ENC_PORT |= (1 << 1);
    
    // Create the state index from previous and current readings
    encoder->old_AB <<= 2;
    encoder->old_AB |= (ENC_PORT & 0x03);
    
    int8_t direction = enc_states[(encoder->old_AB & 0x0f)];
    
    if (direction != 0) {
        uint32_t currentTime = millis();
        uint32_t timeDiff = currentTime - encoder->lastMovementTime;
        
        // Improved acceleration calculation based on rotation speed
        int64_t acceleration = 1;
        if (encoder->_acceleration > 0) {
            if (timeDiff < 30) {
                // Very fast rotation - big jumps
                acceleration = 10;
            } else if (timeDiff < 50) {
                acceleration = 7;
            } else if (timeDiff < 80) {
                acceleration = 5;
            } else if (timeDiff < 120) {
                acceleration = 3;
            } else if (timeDiff < 180) {
                acceleration = 2;
            }
            // Slow rotation (timeDiff >= 180) keeps acceleration = 1
        }
        
        encoder->lastMovementTime = currentTime;
        
        int64_t newPosition = encoder->encoder_position + (direction * acceleration);
        
        // Apply boundaries
        if (encoder->circleValues) {
            if (newPosition > encoder->_maxEncoderValue) {
                newPosition = encoder->_minEncoderValue;
            } else if (newPosition < encoder->_minEncoderValue) {
                newPosition = encoder->_maxEncoderValue;
            }
        } else {
            if (newPosition > encoder->_maxEncoderValue) {
                newPosition = encoder->_maxEncoderValue;
            } else if (newPosition < encoder->_minEncoderValue) {
                newPosition = encoder->_minEncoderValue;
            }
        }
        
        encoder->encoder_position = newPosition;
    }
}

int64_t AiEsp32RotaryEncoder_readEncoder(AiEsp32RotaryEncoder *encoder) {
    return encoder->encoder_position / encoder->encoder_steps;
}

int64_t AiEsp32RotaryEncoder_encoderChanged(AiEsp32RotaryEncoder *encoder) {
    int64_t current = AiEsp32RotaryEncoder_readEncoder(encoder);
    int64_t change = current - encoder->last_read_encoder_position;
    encoder->last_read_encoder_position = current;
    return change;
}

void AiEsp32RotaryEncoder_reset(AiEsp32RotaryEncoder *encoder) {
    encoder->encoder_position = 0;
    encoder->last_read_encoder_position = 0;
}

void AiEsp32RotaryEncoder_setBoundaries(AiEsp32RotaryEncoder *encoder, 
                                         int64_t minValue, 
                                         int64_t maxValue, 
                                         bool circleValues) {
    encoder->_minEncoderValue = minValue * encoder->encoder_steps;
    encoder->_maxEncoderValue = maxValue * encoder->encoder_steps + (encoder->encoder_steps - 1);
    encoder->circleValues = circleValues;
}

void AiEsp32RotaryEncoder_setEncoderValue(AiEsp32RotaryEncoder *encoder, int64_t newValue) {
    encoder->encoder_position = newValue * encoder->encoder_steps;
}

void AiEsp32RotaryEncoder_setAcceleration(AiEsp32RotaryEncoder *encoder, uint16_t acceleration) {
    encoder->_acceleration = acceleration;
}

void AiEsp32RotaryEncoder_disableAcceleration(AiEsp32RotaryEncoder *encoder) {
    encoder->_acceleration = 0;
}

bool AiEsp32RotaryEncoder_isEncoderButtonClicked(AiEsp32RotaryEncoder *encoder) {
    if (encoder->_buttonDisabled || encoder->encoder_button_pin < 0) {
        return false;
    }
    
    bool currentState;
    if (encoder->_isButtonPulldown) {
        currentState = gpio_get_level(encoder->encoder_button_pin) == 1;
    } else {
        currentState = gpio_get_level(encoder->encoder_button_pin) == 0;
    }
    
    uint32_t now = millis();
    
    // Improved debouncing with proper timing
    static uint32_t lastDebounceTime = 0;
    static bool lastStableState = false;
    static bool buttonWasPressed = false;
    
    // Only accept state change if stable for debounce period
    if (currentState != encoder->_currentButtonState) {
        lastDebounceTime = now;
    }
    
    // State has been stable for debounce time
    if ((now - lastDebounceTime) > 30) {  // 30ms debounce
        if (currentState != lastStableState) {
            lastStableState = currentState;
            
            if (currentState) {
                // Button pressed
                buttonWasPressed = true;
            } else if (buttonWasPressed) {
                // Button released after being pressed
                buttonWasPressed = false;
                encoder->_currentButtonState = currentState;
                return true;
            }
        }
    }
    
    encoder->_currentButtonState = currentState;
    return false;
}

bool AiEsp32RotaryEncoder_isEncoderButtonDown(AiEsp32RotaryEncoder *encoder) {
    if (encoder->_buttonDisabled || encoder->encoder_button_pin < 0) {
        return false;
    }
    
    if (encoder->_isButtonPulldown) {
        return gpio_get_level(encoder->encoder_button_pin) == 1;
    } else {
        return gpio_get_level(encoder->encoder_button_pin) == 0;
    }
}

ButtonState AiEsp32RotaryEncoder_currentButtonState(AiEsp32RotaryEncoder *encoder) {
    if (encoder->_buttonDisabled) {
        return BUT_DISABLED;
    }
    
    bool pressed = AiEsp32RotaryEncoder_isEncoderButtonDown(encoder);
    
    if (pressed) {
        if (encoder->_buttonState == BUT_UP || encoder->_buttonState == BUT_RELEASED) {
            encoder->_buttonState = BUT_PUSHED;
        } else {
            encoder->_buttonState = BUT_DOWN;
        }
    } else {
        if (encoder->_buttonState == BUT_DOWN || encoder->_buttonState == BUT_PUSHED) {
            encoder->_buttonState = BUT_RELEASED;
        } else {
            encoder->_buttonState = BUT_UP;
        }
    }
    
    return encoder->_buttonState;
}

void AiEsp32RotaryEncoder_enable(AiEsp32RotaryEncoder *encoder) {
    encoder->_buttonDisabled = false;
}

void AiEsp32RotaryEncoder_disable(AiEsp32RotaryEncoder *encoder) {
    encoder->_buttonDisabled = true;
}

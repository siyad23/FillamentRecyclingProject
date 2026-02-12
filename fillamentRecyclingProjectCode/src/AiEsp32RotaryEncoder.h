/**
 * AiEsp32RotaryEncoder - ESP-IDF Port
 * 
 * This is an ESP-IDF compatible port of the AiEsp32RotaryEncoder library
 * by Igor Antolic (igorantolic/ai-esp32-rotary-encoder)
 * 
 * Original library: https://github.com/igorantolic/ai-esp32-rotary-encoder
 * License: MIT
 */

#ifndef AI_ESP32_ROTARY_ENCODER_H
#define AI_ESP32_ROTARY_ENCODER_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "esp_timer.h"

#ifdef __cplusplus
extern "C" {
#endif

// Button states
typedef enum {
    BUT_DOWN = 0,
    BUT_PUSHED = 1,
    BUT_UP = 2,
    BUT_RELEASED = 3,
    BUT_DISABLED = 99
} ButtonState;

// Rotary encoder structure
typedef struct {
    // Pin configuration
    gpio_num_t encoder_a_pin;
    gpio_num_t encoder_b_pin;
    gpio_num_t encoder_button_pin;
    gpio_num_t encoder_vcc_pin;
    uint8_t encoder_steps;
    
    // Encoder state
    volatile int64_t encoder_position;
    volatile int64_t last_read_encoder_position;
    volatile bool circleValues;
    volatile int64_t _minEncoderValue;
    volatile int64_t _maxEncoderValue;
    
    // Acceleration
    uint16_t _acceleration;
    uint32_t _lastMovementTime;
    int32_t _acceleratedMoveCount;
    
    // Internal state
    volatile int8_t old_AB;
    volatile int64_t lastMovementTime;
    volatile int32_t _pauseLength;
    volatile int8_t _fastRotationLimit;
    
    // Button state
    volatile ButtonState _buttonState;
    volatile uint32_t _lastButtonCheck;
    volatile bool _currentButtonState;
    volatile bool _previousButtonState;
    volatile uint32_t _buttonPressedTime;
    bool _isButtonPulldown;
    bool _buttonDisabled;
    
} AiEsp32RotaryEncoder;

// Constructor/Initialization
void AiEsp32RotaryEncoder_init(AiEsp32RotaryEncoder *encoder,
                                gpio_num_t encoder_a_pin,
                                gpio_num_t encoder_b_pin,
                                gpio_num_t encoder_button_pin,
                                gpio_num_t encoder_vcc_pin,
                                uint8_t encoder_steps);

void AiEsp32RotaryEncoder_initButtonPulldown(AiEsp32RotaryEncoder *encoder,
                                              gpio_num_t encoder_a_pin,
                                              gpio_num_t encoder_b_pin,
                                              gpio_num_t encoder_button_pin,
                                              gpio_num_t encoder_vcc_pin,
                                              uint8_t encoder_steps);

// Setup
void AiEsp32RotaryEncoder_begin(AiEsp32RotaryEncoder *encoder);
void AiEsp32RotaryEncoder_setup(AiEsp32RotaryEncoder *encoder, void (*isr_callback)(void));

// ISR handler - call this from your ISR
void AiEsp32RotaryEncoder_readEncoder_ISR(AiEsp32RotaryEncoder *encoder);

// Encoder reading
int64_t AiEsp32RotaryEncoder_readEncoder(AiEsp32RotaryEncoder *encoder);
int64_t AiEsp32RotaryEncoder_encoderChanged(AiEsp32RotaryEncoder *encoder);
void AiEsp32RotaryEncoder_reset(AiEsp32RotaryEncoder *encoder);

// Boundaries
void AiEsp32RotaryEncoder_setBoundaries(AiEsp32RotaryEncoder *encoder, 
                                         int64_t minValue, 
                                         int64_t maxValue, 
                                         bool circleValues);
void AiEsp32RotaryEncoder_setEncoderValue(AiEsp32RotaryEncoder *encoder, int64_t newValue);

// Acceleration
void AiEsp32RotaryEncoder_setAcceleration(AiEsp32RotaryEncoder *encoder, uint16_t acceleration);
void AiEsp32RotaryEncoder_disableAcceleration(AiEsp32RotaryEncoder *encoder);

// Button functions
bool AiEsp32RotaryEncoder_isEncoderButtonClicked(AiEsp32RotaryEncoder *encoder);
bool AiEsp32RotaryEncoder_isEncoderButtonDown(AiEsp32RotaryEncoder *encoder);
ButtonState AiEsp32RotaryEncoder_currentButtonState(AiEsp32RotaryEncoder *encoder);
void AiEsp32RotaryEncoder_enable(AiEsp32RotaryEncoder *encoder);
void AiEsp32RotaryEncoder_disable(AiEsp32RotaryEncoder *encoder);

#ifdef __cplusplus
}
#endif

#endif // AI_ESP32_ROTARY_ENCODER_H

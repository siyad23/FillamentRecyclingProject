#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "sh1106.h"
#include "AiEsp32RotaryEncoder.h"

static const char *TAG = "MAIN";

// Pin definitions for ESP32-C6 (fixed pins)
#define I2C_MASTER_SCL_IO       20
#define I2C_MASTER_SDA_IO       19
#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_FREQ_HZ      400000

#define ROTARY_ENCODER_A_PIN    15
#define ROTARY_ENCODER_B_PIN    18
#define ROTARY_ENCODER_BUTTON_PIN 14
#define ROTARY_ENCODER_VCC_PIN  -1
#define ROTARY_ENCODER_STEPS    2  // 2 pulses per detent click

// Configurable pin defaults
#define DEFAULT_STEPPER_STEP_PIN    4
#define DEFAULT_STEPPER_DIR_PIN     5
#define DEFAULT_STEPPER_ENABLE_PIN  6
#define DEFAULT_HEATER_PIN          7
#define DEFAULT_MAX6675_CLK_PIN     10
#define DEFAULT_MAX6675_CS_PIN      11
#define DEFAULT_MAX6675_DO_PIN      12

#define SCREEN_ADDRESS          0x3C

// Configurable pins (loaded from NVS)
static int pin_stepper_step = DEFAULT_STEPPER_STEP_PIN;
static int pin_stepper_dir = DEFAULT_STEPPER_DIR_PIN;
static int pin_stepper_enable = DEFAULT_STEPPER_ENABLE_PIN;
static int pin_heater = DEFAULT_HEATER_PIN;
static int pin_max6675_clk = DEFAULT_MAX6675_CLK_PIN;
static int pin_max6675_cs = DEFAULT_MAX6675_CS_PIN;
static int pin_max6675_do = DEFAULT_MAX6675_DO_PIN;

// Display and encoder instances
static sh1106_t display;
static AiEsp32RotaryEncoder rotaryEncoder;

// Menu state machine
typedef enum {
    SPLASH,
    MAIN_MENU,
    START_RUNNING,
    START_CONFIRM_STOP,
    TEMP_SET,
    MOTOR_SPEED_SET,
    MICROSTEP_SET,
    PID_MENU,
    KP_SET,
    KI_SET,
    KD_SET,
    PIN_MENU,
    PIN_STEP_SET,
    PIN_DIR_SET,
    PIN_ENABLE_SET,
    PIN_HEATER_SET,
    PIN_MAX6675_CLK_SET,
    PIN_MAX6675_CS_SET,
    PIN_MAX6675_DO_SET
} menu_state_t;

static menu_state_t current_state = SPLASH;

// Menu variables
static int main_menu_index = 0;
static int start_menu_index = 0;
static int confirm_stop_index = 1;
static int pid_menu_index = 0;
static int pin_menu_index = 0;
static int temperature = 180;  // Default to common PLA temp
static int motor_speed = 50;   // Stepper motor speed (0-100%)
static int microstep = 16;     // Microstep setting (1, 2, 4, 8, 16, 32)
static uint32_t splash_start_time = 0;
static bool needs_update = true;
static int32_t last_encoder_value = 0;
static bool is_running = false;  // Track if process is running
static uint32_t run_start_time = 0;  // Track run time

// Stepper motor control
static uint32_t last_step_time = 0;
static bool heater_enabled = false;

// Animation frame counter
static uint8_t anim_frame = 0;
static uint32_t last_anim_time = 0;

// For slow/fast rotation detection on sliders
static uint32_t last_temp_change_time = 0;
static int32_t temp_encoder_base = 0;  // Base encoder value for temperature

// PID Controller for temperature
static float current_temp = 25.0f;     // Current temperature from MAX6675
static float target_temp = 180.0f;     // Set temperature
static float pid_output = 0.0f;        // PID output (0-100%)
static uint32_t last_max6675_read = 0; // Last MAX6675 read time

// PID constants (stored as int x10 for display, converted to float for calculation)
static int kp_x10 = 20;    // Kp = 2.0
static int ki_x10 = 1;     // Ki = 0.1
static int kd_x10 = 10;    // Kd = 1.0

// PID state variables
static float pid_integral = 0.0f;
static float pid_last_error = 0.0f;
static uint32_t last_pid_time = 0;

// NVS handle
static nvs_handle_t nvs_handle_settings;

// Icon bitmap (24x24) - Recycling symbol
static const uint8_t icon_bmp[] = {
    0x00, 0x3C, 0x00, 0x00, 0x7E, 0x00, 0x01, 0xFF, 0x80, 0x03, 0xC3, 0xC0,
    0x03, 0x81, 0xC0, 0x07, 0x00, 0xE0, 0x1E, 0x00, 0x78, 0x3C, 0x18, 0x3C,
    0x38, 0x3C, 0x1C, 0x30, 0x7E, 0x0C, 0x30, 0xFF, 0x0C, 0x30, 0xFF, 0x0C,
    0x38, 0x3C, 0x1C, 0x3C, 0x18, 0x3C, 0x1E, 0x00, 0x78, 0x07, 0x00, 0xE0,
    0x03, 0x81, 0xC0, 0x03, 0xC3, 0xC0, 0x01, 0xFF, 0x80, 0x00, 0x7E, 0x00,
    0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// Small icons (8x8)
static const uint8_t icon_temp[] = {  // Thermometer
    0x18, 0x24, 0x24, 0x24, 0x24, 0x5A, 0x7E, 0x3C
};
static const uint8_t icon_fan[] = {   // Fan
    0x18, 0x3C, 0x7E, 0xE7, 0xE7, 0x7E, 0x3C, 0x18
};
static const uint8_t icon_play[] = {  // Play arrow
    0x00, 0x60, 0x78, 0x7E, 0x7E, 0x78, 0x60, 0x00
};
static const uint8_t icon_stop[] = {  // Stop square
    0x00, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0x00
};
static const uint8_t icon_back[] = {  // Back arrow
    0x10, 0x30, 0x7E, 0xFF, 0x7E, 0x30, 0x10, 0x00
};
static const uint8_t icon_motor[] = {  // Motor/stepper
    0x3C, 0x42, 0x99, 0xBD, 0xBD, 0x99, 0x42, 0x3C
};
static const uint8_t icon_gear[] = {   // Gear/settings
    0x18, 0x5A, 0xFF, 0xDB, 0xDB, 0xFF, 0x5A, 0x18
};

// Helper function to map value from one range to another
static int map_value(int x, int in_min, int in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// PID Controller calculation
static void update_pid(void) {
    uint32_t now = esp_timer_get_time() / 1000;
    float dt = (now - last_pid_time) / 1000.0f;  // Convert to seconds
    
    if (dt < 0.01f) return;  // Don't update too frequently
    
    // Convert stored int values to float
    float Kp = kp_x10 / 10.0f;
    float Ki = ki_x10 / 10.0f;
    float Kd = kd_x10 / 10.0f;
    
    target_temp = (float)temperature;
    float error = target_temp - current_temp;
    
    // Proportional term
    float p_term = Kp * error;
    
    // Integral term with anti-windup
    pid_integral += error * dt;
    if (pid_integral > 100.0f) pid_integral = 100.0f;
    if (pid_integral < -100.0f) pid_integral = -100.0f;
    float i_term = Ki * pid_integral;
    
    // Derivative term
    float derivative = (error - pid_last_error) / dt;
    float d_term = Kd * derivative;
    
    // Calculate output
    pid_output = p_term + i_term + d_term;
    
    // Clamp output to 0-100%
    if (pid_output > 100.0f) pid_output = 100.0f;
    if (pid_output < 0.0f) pid_output = 0.0f;
    
    // Store for next iteration
    pid_last_error = error;
    last_pid_time = now;
}

// Save settings to NVS flash
static void save_settings_to_nvs(void) {
    nvs_set_i32(nvs_handle_settings, "temperature", temperature);
    nvs_set_i32(nvs_handle_settings, "motor_speed", motor_speed);
    nvs_set_i32(nvs_handle_settings, "microstep", microstep);
    nvs_set_i32(nvs_handle_settings, "kp_x10", kp_x10);
    nvs_set_i32(nvs_handle_settings, "ki_x10", ki_x10);
    nvs_set_i32(nvs_handle_settings, "kd_x10", kd_x10);
    // Save pin configurations
    nvs_set_i32(nvs_handle_settings, "pin_step", pin_stepper_step);
    nvs_set_i32(nvs_handle_settings, "pin_dir", pin_stepper_dir);
    nvs_set_i32(nvs_handle_settings, "pin_enable", pin_stepper_enable);
    nvs_set_i32(nvs_handle_settings, "pin_heater", pin_heater);
    nvs_set_i32(nvs_handle_settings, "pin_clk", pin_max6675_clk);
    nvs_set_i32(nvs_handle_settings, "pin_cs", pin_max6675_cs);
    nvs_set_i32(nvs_handle_settings, "pin_do", pin_max6675_do);
    nvs_commit(nvs_handle_settings);
    ESP_LOGI(TAG, "Settings saved to flash");
}

// Load settings from NVS flash
static void load_settings_from_nvs(void) {
    int32_t val;
    if (nvs_get_i32(nvs_handle_settings, "temperature", &val) == ESP_OK) {
        temperature = val;
    }
    if (nvs_get_i32(nvs_handle_settings, "motor_speed", &val) == ESP_OK) {
        motor_speed = val;
    }
    if (nvs_get_i32(nvs_handle_settings, "microstep", &val) == ESP_OK) {
        microstep = val;
    }
    if (nvs_get_i32(nvs_handle_settings, "kp_x10", &val) == ESP_OK) {
        kp_x10 = val;
    }
    if (nvs_get_i32(nvs_handle_settings, "ki_x10", &val) == ESP_OK) {
        ki_x10 = val;
    }
    if (nvs_get_i32(nvs_handle_settings, "kd_x10", &val) == ESP_OK) {
        kd_x10 = val;
    }
    // Load pin configurations
    if (nvs_get_i32(nvs_handle_settings, "pin_step", &val) == ESP_OK) {
        pin_stepper_step = val;
    }
    if (nvs_get_i32(nvs_handle_settings, "pin_dir", &val) == ESP_OK) {
        pin_stepper_dir = val;
    }
    if (nvs_get_i32(nvs_handle_settings, "pin_enable", &val) == ESP_OK) {
        pin_stepper_enable = val;
    }
    if (nvs_get_i32(nvs_handle_settings, "pin_heater", &val) == ESP_OK) {
        pin_heater = val;
    }
    if (nvs_get_i32(nvs_handle_settings, "pin_clk", &val) == ESP_OK) {
        pin_max6675_clk = val;
    }
    if (nvs_get_i32(nvs_handle_settings, "pin_cs", &val) == ESP_OK) {
        pin_max6675_cs = val;
    }
    if (nvs_get_i32(nvs_handle_settings, "pin_do", &val) == ESP_OK) {
        pin_max6675_do = val;
    }
    ESP_LOGI(TAG, "Settings loaded: Temp=%d, Speed=%d, Microstep=%d",
             temperature, motor_speed, microstep);
}

// Read temperature from MAX6675 (bit-banged SPI)
static float read_max6675_temperature(void) {
    uint16_t data = 0;
    
    // Pull CS low to start communication
    gpio_set_level(pin_max6675_cs, 0);
    esp_rom_delay_us(10);
    
    // Read 16 bits
    for (int i = 15; i >= 0; i--) {
        gpio_set_level(pin_max6675_clk, 0);
        esp_rom_delay_us(10);
        
        if (gpio_get_level(pin_max6675_do)) {
            data |= (1 << i);
        }
        
        gpio_set_level(pin_max6675_clk, 1);
        esp_rom_delay_us(10);
    }
    
    // Pull CS high to end communication
    gpio_set_level(pin_max6675_cs, 1);
    
    // Check for thermocouple open circuit (bit 2)
    if (data & 0x04) {
        ESP_LOGW(TAG, "MAX6675: Thermocouple not connected!");
        return -1.0f;  // Error indicator
    }
    
    // Extract temperature (bits 14:3), each bit = 0.25°C
    data >>= 3;
    float temp = data * 0.25f;
    
    return temp;
}

// Initialize MAX6675 pins
static void init_max6675(void) {
    // Configure CLK and CS as outputs
    gpio_config_t out_conf = {
        .pin_bit_mask = (1ULL << pin_max6675_clk) | (1ULL << pin_max6675_cs),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&out_conf);
    
    // Configure DO as input
    gpio_config_t in_conf = {
        .pin_bit_mask = (1ULL << pin_max6675_do),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&in_conf);
    
    // Set initial states
    gpio_set_level(pin_max6675_cs, 1);  // CS high (inactive)
    gpio_set_level(pin_max6675_clk, 1); // CLK high
    
    ESP_LOGI(TAG, "MAX6675 initialized: CLK=%d, CS=%d, DO=%d", 
             pin_max6675_clk, pin_max6675_cs, pin_max6675_do);
}

// Initialize hardware control pins
static void init_hardware_control(void) {
    // Configure stepper motor pins
    gpio_config_t stepper_conf = {
        .pin_bit_mask = (1ULL << pin_stepper_step) | (1ULL << pin_stepper_dir) | (1ULL << pin_stepper_enable),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&stepper_conf);
    
    // Disable stepper initially (HIGH = disabled for TB6600)
    gpio_set_level(pin_stepper_enable, 1);
    gpio_set_level(pin_stepper_dir, 0);
    gpio_set_level(pin_stepper_step, 0);
    
    // Configure heater pin
    gpio_config_t heater_conf = {
        .pin_bit_mask = (1ULL << pin_heater),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&heater_conf);
    gpio_set_level(pin_heater, 0);  // Heater off initially
    
    ESP_LOGI(TAG, "Hardware pins initialized: STEP=%d, DIR=%d, EN=%d, HEATER=%d", 
             pin_stepper_step, pin_stepper_dir, pin_stepper_enable, pin_heater);
}

// Start the process - enable heater and motor
static void start_process(void) {
    is_running = true;
    run_start_time = esp_timer_get_time() / 1000;
    pid_integral = 0.0f;  // Reset PID integral
    pid_last_error = 0.0f;
    last_pid_time = esp_timer_get_time() / 1000;
    
    // Enable stepper motor (LOW = enabled for TB6600)
    gpio_set_level(pin_stepper_enable, 0);
    gpio_set_level(pin_stepper_dir, 1);  // Set direction
    
    heater_enabled = true;
    ESP_LOGI(TAG, "Process started - Heater and motor enabled");
}

// Stop the process - disable heater and motor
static void stop_process(void) {
    is_running = false;
    heater_enabled = false;
    
    // Disable stepper motor (HIGH = disabled for TB6600)
    gpio_set_level(pin_stepper_enable, 1);
    gpio_set_level(pin_stepper_step, 0);
    
    // Turn off heater
    gpio_set_level(pin_heater, 0);
    pid_output = 0.0f;
    
    ESP_LOGI(TAG, "Process stopped - Heater and motor disabled");
}

// Update heater PWM based on PID output
static void update_heater_output(void) {
    if (heater_enabled && is_running) {
        // Simple on/off control based on PID output percentage
        // For better control, use LEDC PWM
        if (pid_output > 50.0f) {
            gpio_set_level(pin_heater, 1);
        } else {
            gpio_set_level(pin_heater, 0);
        }
    } else {
        gpio_set_level(pin_heater, 0);
    }
}

// Generate step pulses for stepper motor
static void update_stepper_motor(void) {
    if (!is_running || motor_speed == 0) {
        return;
    }
    
    // Calculate step interval based on speed and microstep
    // Higher microstep = smoother but needs more pulses
    // Speed 100% = fastest, Speed 1% = slowest
    uint32_t base_interval_us = 500;  // Base step interval at 100% speed
    uint32_t max_interval_us = 50000; // Slowest step interval at 1% speed
    
    // Calculate interval: slower speed = longer interval
    uint32_t step_interval_us = base_interval_us + 
        ((100 - motor_speed) * (max_interval_us - base_interval_us)) / 100;
    
    uint32_t now_us = esp_timer_get_time();
    if ((now_us - last_step_time) >= step_interval_us) {
        // Generate step pulse
        gpio_set_level(pin_stepper_step, 1);
        esp_rom_delay_us(5);  // 5us pulse width (TB6600 needs min 2.5us)
        gpio_set_level(pin_stepper_step, 0);
        last_step_time = now_us;
    }
}

// Format time as MM:SS
static void format_time(uint32_t seconds, char *buffer, size_t size) {
    uint32_t mins = seconds / 60;
    uint32_t secs = seconds % 60;
    snprintf(buffer, size, "%02lu:%02lu", (unsigned long)mins, (unsigned long)secs);
}

// Draw a small 8x8 bitmap icon
static void draw_icon_8x8(int16_t x, int16_t y, const uint8_t *bitmap, bool invert) {
    for (int row = 0; row < 8; row++) {
        uint8_t line = bitmap[row];
        for (int col = 0; col < 8; col++) {
            if (line & (0x80 >> col)) {
                sh1106_draw_pixel(&display, x + col, y + row, 
                    invert ? SH1106_COLOR_BLACK : SH1106_COLOR_WHITE);
            } else if (invert) {
                sh1106_draw_pixel(&display, x + col, y + row, SH1106_COLOR_WHITE);
            }
        }
    }
}

// ISR callback for rotary encoder
void IRAM_ATTR readEncoderISR(void) {
    AiEsp32RotaryEncoder_readEncoder_ISR(&rotaryEncoder);
}

static void configure_encoder_for_menu(int min, int max, int value, bool acceleration) {
    AiEsp32RotaryEncoder_setBoundaries(&rotaryEncoder, min, max, !acceleration);
    AiEsp32RotaryEncoder_setEncoderValue(&rotaryEncoder, value);
    // Enable acceleration for sliders (fast rotation = bigger jumps)
    AiEsp32RotaryEncoder_setAcceleration(&rotaryEncoder, acceleration ? 150 : 0);
    last_encoder_value = value;
}

static void draw_main_menu(void) {
    // Header with border
    sh1106_fill_rect(&display, 0, 0, 128, 12, SH1106_COLOR_WHITE);
    sh1106_set_text_size(&display, 1);
    sh1106_set_text_color(&display, SH1106_COLOR_BLACK, SH1106_COLOR_WHITE);
    sh1106_set_cursor(&display, 20, 2);
    sh1106_print(&display, "FILAMENT RECYCLER");

    // Menu items (6 items, show 3 at a time with scrolling)
    const char *items[] = {"Start Process", "Temperature", "Motor Speed", "Microstep", "PID Settings", "Pin Settings"};
    const uint8_t *icons[] = {icon_play, icon_temp, icon_motor, icon_gear, icon_gear, icon_gear};
    
    // Calculate scroll offset to keep selected item visible
    int scroll_offset = 0;
    if (main_menu_index > 2) {
        scroll_offset = main_menu_index - 2;
    }
    
    for (int i = 0; i < 3; i++) {
        int item_idx = i + scroll_offset;
        if (item_idx >= 6) break;
        
        int y = 16 + (i * 16);
        
        if (item_idx == main_menu_index) {
            // Selected item - inverted
            sh1106_fill_rect(&display, 0, y, 128, 14, SH1106_COLOR_WHITE);
            sh1106_set_text_color(&display, SH1106_COLOR_BLACK, SH1106_COLOR_WHITE);
            draw_icon_8x8(4, y + 3, icons[item_idx], true);
            sh1106_set_cursor(&display, 16, y + 3);
            sh1106_print(&display, items[item_idx]);
            
            // Show current value
            char val[12];
            if (item_idx == 1) {
                snprintf(val, sizeof(val), "%dC", temperature);
                sh1106_set_cursor(&display, 100, y + 3);
                sh1106_print(&display, val);
            } else if (item_idx == 2) {
                snprintf(val, sizeof(val), "%d%%", motor_speed);
                sh1106_set_cursor(&display, 100, y + 3);
                sh1106_print(&display, val);
            } else if (item_idx == 3) {
                snprintf(val, sizeof(val), "1/%d", microstep);
                sh1106_set_cursor(&display, 96, y + 3);
                sh1106_print(&display, val);
            }
            
            sh1106_set_cursor(&display, 120, y + 3);
            sh1106_print(&display, ">");
        } else {
            sh1106_set_text_color(&display, SH1106_COLOR_WHITE, SH1106_COLOR_BLACK);
            draw_icon_8x8(4, y + 3, icons[item_idx], false);
            sh1106_set_cursor(&display, 16, y + 3);
            sh1106_print(&display, items[item_idx]);
            
            // Show current value
            char val[12];
            if (item_idx == 1) {
                snprintf(val, sizeof(val), "%dC", temperature);
                sh1106_set_cursor(&display, 100, y + 3);
                sh1106_print(&display, val);
            } else if (item_idx == 2) {
                snprintf(val, sizeof(val), "%d%%", motor_speed);
                sh1106_set_cursor(&display, 100, y + 3);
                sh1106_print(&display, val);
            } else if (item_idx == 3) {
                snprintf(val, sizeof(val), "1/%d", microstep);
                sh1106_set_cursor(&display, 96, y + 3);
                sh1106_print(&display, val);
            }
        }
    }
    
    // Scroll indicators
    if (scroll_offset > 0) {
        sh1106_set_cursor(&display, 60, 14);
        sh1106_set_text_color(&display, SH1106_COLOR_WHITE, SH1106_COLOR_BLACK);
        sh1106_print(&display, "^");
    }
    if (scroll_offset + 3 < 6) {
        sh1106_set_cursor(&display, 60, 62);
        sh1106_set_text_color(&display, SH1106_COLOR_WHITE, SH1106_COLOR_BLACK);
        sh1106_print(&display, "v");
    }
}

static void draw_start_running(void) {
    // Header
    sh1106_fill_rect(&display, 0, 0, 128, 12, SH1106_COLOR_WHITE);
    sh1106_set_text_size(&display, 1);
    sh1106_set_text_color(&display, SH1106_COLOR_BLACK, SH1106_COLOR_WHITE);
    sh1106_set_cursor(&display, 30, 2);
    sh1106_print(&display, "RUNNING");
    
    // Animated spinner
    const char spinner[] = {'|', '/', '-', '\\'};
    char spin_str[2] = {spinner[anim_frame % 4], '\0'};
    sh1106_set_cursor(&display, 80, 2);
    sh1106_print(&display, spin_str);

    sh1106_set_text_color(&display, SH1106_COLOR_WHITE, SH1106_COLOR_BLACK);
    
    // Temperature display - Current and Set
    draw_icon_8x8(4, 15, icon_temp, false);
    sh1106_set_cursor(&display, 16, 16);
    char curr_temp_str[16];
    snprintf(curr_temp_str, sizeof(curr_temp_str), "%.0fC", current_temp);
    sh1106_print(&display, curr_temp_str);
    
    sh1106_set_cursor(&display, 50, 16);
    sh1106_print(&display, "->");
    
    sh1106_set_cursor(&display, 68, 16);
    char set_temp_str[16];
    snprintf(set_temp_str, sizeof(set_temp_str), "%dC", temperature);
    sh1106_print(&display, set_temp_str);
    
    // PID output (heater power)
    sh1106_set_cursor(&display, 100, 16);
    char pid_str[8];
    snprintf(pid_str, sizeof(pid_str), "%d%%", (int)pid_output);
    sh1106_print(&display, pid_str);
    
    // Motor speed
    draw_icon_8x8(4, 27, icon_motor, false);
    sh1106_set_cursor(&display, 16, 28);
    char speed_str[16];
    snprintf(speed_str, sizeof(speed_str), "M:%d%%", motor_speed);
    sh1106_print(&display, speed_str);
    
    // Run time
    uint32_t elapsed = (esp_timer_get_time() / 1000 - run_start_time) / 1000;
    char time_str[16];
    format_time(elapsed, time_str, sizeof(time_str));
    sh1106_set_cursor(&display, 60, 28);
    sh1106_print(&display, "T:");
    sh1106_print(&display, time_str);
    
    // Temperature progress bar (shows how close current is to target)
    sh1106_draw_rect(&display, 4, 40, 120, 8, SH1106_COLOR_WHITE);
    int temp_progress = 0;
    if (temperature > 30) {
        temp_progress = (int)((current_temp - 30) * 116 / (temperature - 30));
        if (temp_progress < 0) temp_progress = 0;
        if (temp_progress > 116) temp_progress = 116;
    }
    if (temp_progress > 0) {
        sh1106_fill_rect(&display, 6, 42, temp_progress, 4, SH1106_COLOR_WHITE);
    }
    
    // Bottom buttons
    sh1106_draw_line(&display, 0, 52, 128, 52, SH1106_COLOR_WHITE);
    
    const char *items[] = {"BACK", "STOP"};
    for (int i = 0; i < 2; i++) {
        int x = (i == 0) ? 4 : 80;
        
        if (i == start_menu_index) {
            sh1106_fill_rect(&display, x - 2, 54, 46, 10, SH1106_COLOR_WHITE);
            sh1106_set_text_color(&display, SH1106_COLOR_BLACK, SH1106_COLOR_WHITE);
        } else {
            sh1106_set_text_color(&display, SH1106_COLOR_WHITE, SH1106_COLOR_BLACK);
        }
        
        draw_icon_8x8(x, 55, (i == 0) ? icon_back : icon_stop, i == start_menu_index);
        sh1106_set_cursor(&display, x + 10, 55);
        sh1106_print(&display, items[i]);
    }
}

static void draw_confirm_stop(void) {
    // Warning dialog box
    sh1106_draw_rect(&display, 10, 8, 108, 48, SH1106_COLOR_WHITE);
    sh1106_draw_rect(&display, 11, 9, 106, 46, SH1106_COLOR_WHITE);
    
    // Title
    sh1106_fill_rect(&display, 12, 10, 104, 12, SH1106_COLOR_WHITE);
    sh1106_set_text_size(&display, 1);
    sh1106_set_text_color(&display, SH1106_COLOR_BLACK, SH1106_COLOR_WHITE);
    sh1106_set_cursor(&display, 30, 12);
    sh1106_print(&display, "STOP PROCESS?");
    
    sh1106_set_text_color(&display, SH1106_COLOR_WHITE, SH1106_COLOR_BLACK);
    sh1106_set_cursor(&display, 20, 26);
    sh1106_print(&display, "Are you sure?");

    // Buttons
    const char *items[] = {"YES", "NO"};
    for (int i = 0; i < 2; i++) {
        int x = (i == 0) ? 20 : 75;
        
        if (i == confirm_stop_index) {
            sh1106_fill_rect(&display, x, 40, 32, 12, SH1106_COLOR_WHITE);
            sh1106_set_text_color(&display, SH1106_COLOR_BLACK, SH1106_COLOR_WHITE);
        } else {
            sh1106_draw_rect(&display, x, 40, 32, 12, SH1106_COLOR_WHITE);
            sh1106_set_text_color(&display, SH1106_COLOR_WHITE, SH1106_COLOR_BLACK);
        }
        
        sh1106_set_cursor(&display, x + 8, 42);
        sh1106_print(&display, items[i]);
    }
}

static void draw_slider(const char *title, int val, int min_v, int max_v, const char *unit, const uint8_t *icon) {
    // Header
    sh1106_fill_rect(&display, 0, 0, 128, 12, SH1106_COLOR_WHITE);
    sh1106_set_text_size(&display, 1);
    sh1106_set_text_color(&display, SH1106_COLOR_BLACK, SH1106_COLOR_WHITE);
    sh1106_set_cursor(&display, 4, 2);
    sh1106_print(&display, title);
    
    // Back hint
    sh1106_set_cursor(&display, 90, 2);
    sh1106_print(&display, "[CLICK]");

    sh1106_set_text_color(&display, SH1106_COLOR_WHITE, SH1106_COLOR_BLACK);
    
    // Icon and large value display
    draw_icon_8x8(10, 20, icon, false);
    
    sh1106_set_text_size(&display, 2);
    sh1106_set_cursor(&display, 30, 18);
    char val_str[16];
    snprintf(val_str, sizeof(val_str), "%d", val);
    sh1106_print(&display, val_str);
    
    sh1106_set_text_size(&display, 1);
    sh1106_set_cursor(&display, 30 + strlen(val_str) * 12, 24);
    sh1106_print(&display, unit);
    
    // Min/max labels
    char min_str[8], max_str[8];
    snprintf(min_str, sizeof(min_str), "%d", min_v);
    snprintf(max_str, sizeof(max_str), "%d", max_v);
    sh1106_set_cursor(&display, 4, 42);
    sh1106_print(&display, min_str);
    sh1106_set_cursor(&display, 110, 42);
    sh1106_print(&display, max_str);

    // Progress bar with border
    sh1106_draw_rect(&display, 4, 52, 120, 10, SH1106_COLOR_WHITE);
    int w = map_value(val, min_v, max_v, 0, 116);
    if (w > 0) {
        sh1106_fill_rect(&display, 6, 54, w, 6, SH1106_COLOR_WHITE);
    }
    
    // Tick marks
    for (int i = 0; i <= 4; i++) {
        int x = 4 + (i * 30);
        sh1106_draw_line(&display, x, 50, x, 52, SH1106_COLOR_WHITE);
    }
}

// Draw slider for float values (PID gains shown as x.x)
static void draw_float_slider(const char *title, int val_x10, int min_x10, int max_x10, const char *label) {
    // Header
    sh1106_fill_rect(&display, 0, 0, 128, 12, SH1106_COLOR_WHITE);
    sh1106_set_text_size(&display, 1);
    sh1106_set_text_color(&display, SH1106_COLOR_BLACK, SH1106_COLOR_WHITE);
    sh1106_set_cursor(&display, 4, 2);
    sh1106_print(&display, title);
    
    // Back hint
    sh1106_set_cursor(&display, 90, 2);
    sh1106_print(&display, "[CLICK]");

    sh1106_set_text_color(&display, SH1106_COLOR_WHITE, SH1106_COLOR_BLACK);
    
    // Large value display (as float x.x)
    sh1106_set_text_size(&display, 2);
    sh1106_set_cursor(&display, 30, 20);
    char val_str[16];
    snprintf(val_str, sizeof(val_str), "%d.%d", val_x10 / 10, val_x10 % 10);
    sh1106_print(&display, val_str);
    
    sh1106_set_text_size(&display, 1);
    
    // Min/max labels
    char min_str[8], max_str[8];
    snprintf(min_str, sizeof(min_str), "%d.%d", min_x10 / 10, min_x10 % 10);
    snprintf(max_str, sizeof(max_str), "%d.%d", max_x10 / 10, max_x10 % 10);
    sh1106_set_cursor(&display, 4, 42);
    sh1106_print(&display, min_str);
    sh1106_set_cursor(&display, 100, 42);
    sh1106_print(&display, max_str);

    // Progress bar
    sh1106_draw_rect(&display, 4, 52, 120, 10, SH1106_COLOR_WHITE);
    int w = map_value(val_x10, min_x10, max_x10, 0, 116);
    if (w > 0) {
        sh1106_fill_rect(&display, 6, 54, w, 6, SH1106_COLOR_WHITE);
    }
}

static void draw_pid_menu(void) {
    // Header
    sh1106_fill_rect(&display, 0, 0, 128, 12, SH1106_COLOR_WHITE);
    sh1106_set_text_size(&display, 1);
    sh1106_set_text_color(&display, SH1106_COLOR_BLACK, SH1106_COLOR_WHITE);
    sh1106_set_cursor(&display, 30, 2);
    sh1106_print(&display, "PID SETTINGS");

    // Menu items
    const char *items[] = {"Kp (Prop)", "Ki (Int)", "Kd (Deriv)", "< Back"};
    int *values[] = {&kp_x10, &ki_x10, &kd_x10, NULL};
    
    for (int i = 0; i < 4; i++) {
        int y = 16 + (i * 12);
        
        if (i == pid_menu_index) {
            sh1106_fill_rect(&display, 0, y, 128, 11, SH1106_COLOR_WHITE);
            sh1106_set_text_color(&display, SH1106_COLOR_BLACK, SH1106_COLOR_WHITE);
        } else {
            sh1106_set_text_color(&display, SH1106_COLOR_WHITE, SH1106_COLOR_BLACK);
        }
        
        sh1106_set_cursor(&display, 4, y + 2);
        sh1106_print(&display, items[i]);
        
        // Show current value for Kp/Ki/Kd
        if (values[i] != NULL) {
            char val[12];
            snprintf(val, sizeof(val), "%d.%d", *values[i] / 10, *values[i] % 10);
            sh1106_set_cursor(&display, 90, y + 2);
            sh1106_print(&display, val);
        }
    }
}

static void draw_pin_menu(void) {
    // Header
    sh1106_fill_rect(&display, 0, 0, 128, 12, SH1106_COLOR_WHITE);
    sh1106_set_text_size(&display, 1);
    sh1106_set_text_color(&display, SH1106_COLOR_BLACK, SH1106_COLOR_WHITE);
    sh1106_set_cursor(&display, 30, 2);
    sh1106_print(&display, "PIN SETTINGS");

    // Menu items (8 total, show 4 at a time with scrolling)
    const char *items[] = {"Step Pin", "Dir Pin", "Enable Pin", "Heater Pin", 
                           "MAX CLK", "MAX CS", "MAX DO", "< Back"};
    int values[] = {pin_stepper_step, pin_stepper_dir, pin_stepper_enable, pin_heater,
                    pin_max6675_clk, pin_max6675_cs, pin_max6675_do, -1};
    
    int scroll_offset = 0;
    if (pin_menu_index > 3) scroll_offset = pin_menu_index - 3;
    if (scroll_offset > 4) scroll_offset = 4;  // Max scroll (8 items - 4 visible)
    
    for (int i = 0; i < 4; i++) {
        int item_idx = i + scroll_offset;
        if (item_idx >= 8) break;
        
        int y = 16 + (i * 12);
        
        if (item_idx == pin_menu_index) {
            sh1106_fill_rect(&display, 0, y, 128, 11, SH1106_COLOR_WHITE);
            sh1106_set_text_color(&display, SH1106_COLOR_BLACK, SH1106_COLOR_WHITE);
        } else {
            sh1106_set_text_color(&display, SH1106_COLOR_WHITE, SH1106_COLOR_BLACK);
        }
        
        sh1106_set_cursor(&display, 4, y + 2);
        sh1106_print(&display, items[item_idx]);
        
        // Show GPIO number for pin items
        if (values[item_idx] >= 0) {
            char val[8];
            snprintf(val, sizeof(val), "GPIO%d", values[item_idx]);
            sh1106_set_cursor(&display, 85, y + 2);
            sh1106_print(&display, val);
        }
    }
    
    // Scroll indicators
    if (scroll_offset > 0) {
        sh1106_set_cursor(&display, 60, 14);
        sh1106_set_text_color(&display, SH1106_COLOR_WHITE, SH1106_COLOR_BLACK);
        sh1106_print(&display, "^");
    }
    if (scroll_offset + 4 < 8) {
        sh1106_set_cursor(&display, 60, 62);
        sh1106_set_text_color(&display, SH1106_COLOR_WHITE, SH1106_COLOR_BLACK);
        sh1106_print(&display, "v");
    }
}

// Draw pin setting slider
static void draw_pin_slider(const char *title, int pin_value) {
    // Header
    sh1106_fill_rect(&display, 0, 0, 128, 12, SH1106_COLOR_WHITE);
    sh1106_set_text_size(&display, 1);
    sh1106_set_text_color(&display, SH1106_COLOR_BLACK, SH1106_COLOR_WHITE);
    sh1106_set_cursor(&display, 4, 2);
    sh1106_print(&display, title);
    
    // Back hint
    sh1106_set_cursor(&display, 90, 2);
    sh1106_print(&display, "[CLICK]");

    sh1106_set_text_color(&display, SH1106_COLOR_WHITE, SH1106_COLOR_BLACK);
    
    // Large value display
    sh1106_set_text_size(&display, 2);
    sh1106_set_cursor(&display, 30, 22);
    char val_str[16];
    snprintf(val_str, sizeof(val_str), "GPIO %d", pin_value);
    sh1106_print(&display, val_str);
    
    sh1106_set_text_size(&display, 1);
    
    // Range info
    sh1106_set_cursor(&display, 20, 45);
    sh1106_print(&display, "Range: 0 - 21");
    
    // Progress bar
    sh1106_draw_rect(&display, 4, 52, 120, 10, SH1106_COLOR_WHITE);
    int w = map_value(pin_value, 0, 21, 0, 116);
    if (w > 0) {
        sh1106_fill_rect(&display, 6, 54, w, 6, SH1106_COLOR_WHITE);
    }
}

static void handle_menu_click(void) {
    switch (current_state) {
        case MAIN_MENU:
            if (main_menu_index == 0) {
                current_state = START_RUNNING;
                start_process();  // Enable heater and motor
                configure_encoder_for_menu(0, 1, 0, false);
            } else if (main_menu_index == 1) {
                current_state = TEMP_SET;
                // For temperature: use unbounded encoder, we'll manage the value ourselves
                AiEsp32RotaryEncoder_setBoundaries(&rotaryEncoder, -10000, 10000, false);
                AiEsp32RotaryEncoder_setEncoderValue(&rotaryEncoder, 0);
                AiEsp32RotaryEncoder_setAcceleration(&rotaryEncoder, 0);  // No library acceleration
                temp_encoder_base = 0;
                last_encoder_value = 0;
                last_temp_change_time = esp_timer_get_time() / 1000;
            } else if (main_menu_index == 2) {
                current_state = MOTOR_SPEED_SET;
                configure_encoder_for_menu(0, 100, motor_speed, true);
            } else if (main_menu_index == 3) {
                current_state = MICROSTEP_SET;
                // Microstep values: 0=1, 1=2, 2=4, 3=8, 4=16, 5=32
                int microstep_idx = 0;
                if (microstep == 2) microstep_idx = 1;
                else if (microstep == 4) microstep_idx = 2;
                else if (microstep == 8) microstep_idx = 3;
                else if (microstep == 16) microstep_idx = 4;
                else if (microstep == 32) microstep_idx = 5;
                configure_encoder_for_menu(0, 5, microstep_idx, false);
            } else if (main_menu_index == 4) {
                current_state = PID_MENU;
                configure_encoder_for_menu(0, 3, 0, false);
            } else if (main_menu_index == 5) {
                current_state = PIN_MENU;
                configure_encoder_for_menu(0, 7, 0, false);  // 8 items (0-7)
            }
            break;

        case START_RUNNING:
            if (start_menu_index == 0) {
                // Back - stop the process
                stop_process();  // Disable heater and motor
                current_state = MAIN_MENU;
                configure_encoder_for_menu(0, 5, 0, false);
            } else {
                current_state = START_CONFIRM_STOP;
                configure_encoder_for_menu(0, 1, 1, false);  // Default to NO
            }
            break;

        case START_CONFIRM_STOP:
            if (confirm_stop_index == 0) {  // YES
                stop_process();  // Disable heater and motor
                current_state = MAIN_MENU;
                configure_encoder_for_menu(0, 5, 0, false);
            } else {  // NO
                current_state = START_RUNNING;
                configure_encoder_for_menu(0, 1, 0, false);
            }
            break;

        case TEMP_SET:
            save_settings_to_nvs();
            current_state = MAIN_MENU;
            configure_encoder_for_menu(0, 5, 1, false);
            break;

        case MOTOR_SPEED_SET:
            save_settings_to_nvs();
            current_state = MAIN_MENU;
            configure_encoder_for_menu(0, 5, 2, false);
            break;

        case MICROSTEP_SET:
            save_settings_to_nvs();
            current_state = MAIN_MENU;
            configure_encoder_for_menu(0, 5, 3, false);
            break;

        case PID_MENU:
            if (pid_menu_index == 0) {
                current_state = KP_SET;
                configure_encoder_for_menu(0, 100, kp_x10, true);  // 0.0 to 10.0
            } else if (pid_menu_index == 1) {
                current_state = KI_SET;
                configure_encoder_for_menu(0, 50, ki_x10, true);   // 0.0 to 5.0
            } else if (pid_menu_index == 2) {
                current_state = KD_SET;
                configure_encoder_for_menu(0, 100, kd_x10, true);  // 0.0 to 10.0
            } else {
                // Back
                current_state = MAIN_MENU;
                configure_encoder_for_menu(0, 5, 4, false);
            }
            break;

        case KP_SET:
            save_settings_to_nvs();
            current_state = PID_MENU;
            configure_encoder_for_menu(0, 3, 0, false);
            break;

        case KI_SET:
            save_settings_to_nvs();
            current_state = PID_MENU;
            configure_encoder_for_menu(0, 3, 1, false);
            break;

        case KD_SET:
            save_settings_to_nvs();
            current_state = PID_MENU;
            configure_encoder_for_menu(0, 3, 2, false);
            break;

        case PIN_MENU:
            if (pin_menu_index == 0) {
                current_state = PIN_STEP_SET;
                configure_encoder_for_menu(0, 21, pin_stepper_step, false);
            } else if (pin_menu_index == 1) {
                current_state = PIN_DIR_SET;
                configure_encoder_for_menu(0, 21, pin_stepper_dir, false);
            } else if (pin_menu_index == 2) {
                current_state = PIN_ENABLE_SET;
                configure_encoder_for_menu(0, 21, pin_stepper_enable, false);
            } else if (pin_menu_index == 3) {
                current_state = PIN_HEATER_SET;
                configure_encoder_for_menu(0, 21, pin_heater, false);
            } else if (pin_menu_index == 4) {
                current_state = PIN_MAX6675_CLK_SET;
                configure_encoder_for_menu(0, 21, pin_max6675_clk, false);
            } else if (pin_menu_index == 5) {
                current_state = PIN_MAX6675_CS_SET;
                configure_encoder_for_menu(0, 21, pin_max6675_cs, false);
            } else if (pin_menu_index == 6) {
                current_state = PIN_MAX6675_DO_SET;
                configure_encoder_for_menu(0, 21, pin_max6675_do, false);
            } else {
                // Back
                current_state = MAIN_MENU;
                configure_encoder_for_menu(0, 5, 5, false);
            }
            break;

        case PIN_STEP_SET:
        case PIN_DIR_SET:
        case PIN_ENABLE_SET:
        case PIN_HEATER_SET:
        case PIN_MAX6675_CLK_SET:
        case PIN_MAX6675_CS_SET:
        case PIN_MAX6675_DO_SET:
            save_settings_to_nvs();
            current_state = PIN_MENU;
            configure_encoder_for_menu(0, 7, pin_menu_index, false);
            break;

        default:
            break;
    }
}

static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) return ret;
    
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void app_main(void) {
    ESP_LOGI(TAG, "Filament Recycling Project Starting...");

    // Initialize NVS (Non-Volatile Storage)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Open NVS handle
    ret = nvs_open("settings", NVS_READWRITE, &nvs_handle_settings);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle");
    } else {
        // Load saved settings
        load_settings_from_nvs();
    }

    // Initialize I2C
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized");

    // Initialize hardware control (stepper motor, heater)
    init_hardware_control();
    
    // Initialize MAX6675 temperature sensor
    init_max6675();

    // Initialize rotary encoder using AiEsp32RotaryEncoder
    AiEsp32RotaryEncoder_init(&rotaryEncoder, 
                               ROTARY_ENCODER_A_PIN, 
                               ROTARY_ENCODER_B_PIN, 
                               ROTARY_ENCODER_BUTTON_PIN, 
                               ROTARY_ENCODER_VCC_PIN, 
                               ROTARY_ENCODER_STEPS);
    AiEsp32RotaryEncoder_begin(&rotaryEncoder);
    AiEsp32RotaryEncoder_setup(&rotaryEncoder, readEncoderISR);
    AiEsp32RotaryEncoder_setBoundaries(&rotaryEncoder, 0, 4, true);
    AiEsp32RotaryEncoder_setAcceleration(&rotaryEncoder, 0);

    // Initialize display
    ret = sh1106_init(&display, I2C_MASTER_NUM, SCREEN_ADDRESS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SH1106 initialization failed");
        return;
    }

    sh1106_clear(&display);
    current_state = SPLASH;
    splash_start_time = esp_timer_get_time() / 1000;  // Convert to milliseconds

    // Main loop
    while (1) {
        uint32_t current_time = esp_timer_get_time() / 1000;

        // Update animation frame (every 200ms)
        if (current_time - last_anim_time > 200) {
            anim_frame++;
            last_anim_time = current_time;
            if (current_state == START_RUNNING) {
                needs_update = true;  // Force redraw for animation
            }
        }
        
        // Read MAX6675 temperature sensor (every 250ms - sensor needs ~220ms conversion time)
        if (current_time - last_max6675_read > 250) {
            float temp = read_max6675_temperature();
            if (temp >= 0) {  // Valid reading (negative means error)
                current_temp = temp;
            }
            last_max6675_read = current_time;
        }

        // Update PID controller when running
        if (is_running) {
            update_pid();
            update_heater_output();
            update_stepper_motor();
        }

        if (current_state == SPLASH) {
            sh1106_clear(&display);
            
            // Draw border
            sh1106_draw_rect(&display, 0, 0, 128, 64, SH1106_COLOR_WHITE);
            
            // Draw logo
            sh1106_draw_bitmap(&display, (128 - 24) / 2, 8, icon_bmp, 24, 24, SH1106_COLOR_WHITE);
            
            sh1106_set_text_size(&display, 1);
            sh1106_set_text_color(&display, SH1106_COLOR_WHITE, SH1106_COLOR_BLACK);
            sh1106_set_cursor(&display, 20, 38);
            sh1106_print(&display, "FILAMENT RECYCLER");
            
            // Loading bar animation
            int load_width = ((current_time - splash_start_time) * 100) / 2000;
            if (load_width > 100) load_width = 100;
            sh1106_draw_rect(&display, 14, 52, 100, 6, SH1106_COLOR_WHITE);
            sh1106_fill_rect(&display, 16, 54, (load_width * 96) / 100, 2, SH1106_COLOR_WHITE);
            
            sh1106_display(&display);

            if (current_time - splash_start_time > 2000) {
                current_state = MAIN_MENU;
                configure_encoder_for_menu(0, 5, 0, false);
                needs_update = true;
            }
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        // Read encoder value
        int32_t new_encoder_value = AiEsp32RotaryEncoder_readEncoder(&rotaryEncoder);
        if (new_encoder_value != last_encoder_value) {
            last_encoder_value = new_encoder_value;
            needs_update = true;

            switch (current_state) {
                case MAIN_MENU:
                    main_menu_index = new_encoder_value;
                    break;
                case START_RUNNING:
                    start_menu_index = new_encoder_value;
                    break;
                case START_CONFIRM_STOP:
                    confirm_stop_index = new_encoder_value;
                    break;
                case TEMP_SET: {
                    // Manual handling: detect rotation speed and apply appropriate step
                    // Divide by ROTARY_ENCODER_STEPS to get actual detent clicks
                    int32_t raw_delta = new_encoder_value - temp_encoder_base;
                    int32_t delta = raw_delta / ROTARY_ENCODER_STEPS;
                    if (delta != 0) {
                        uint32_t now = esp_timer_get_time() / 1000;
                        uint32_t time_since_last = now - last_temp_change_time;
                        
                        // Determine step size based on rotation speed
                        int step;
                        if (time_since_last >= 500) {
                            // Slow rotation (~1 per second or slower): 1 degree
                            step = 1;
                        } else if (time_since_last >= 200) {
                            // Medium rotation: 5 degrees
                            step = 5;
                        } else if (time_since_last >= 100) {
                            // Fast rotation: 10 degrees
                            step = 10;
                        } else {
                            // Very fast rotation: 20 degrees
                            step = 20;
                        }
                        
                        // Apply change based on direction
                        if (delta > 0) {
                            temperature += step;
                        } else {
                            temperature -= step;
                        }
                        
                        // Clamp to valid range
                        if (temperature < 30) temperature = 30;
                        if (temperature > 250) temperature = 250;
                        
                        // Reset base by the actual detent movement (not raw encoder value)
                        temp_encoder_base += delta * ROTARY_ENCODER_STEPS;
                        last_temp_change_time = now;
                    }
                    break;
                }
                case MOTOR_SPEED_SET:
                    motor_speed = new_encoder_value;
                    break;
                case MICROSTEP_SET: {
                    // Convert encoder value to microstep: 0=1, 1=2, 2=4, 3=8, 4=16, 5=32
                    int microstep_values[] = {1, 2, 4, 8, 16, 32};
                    microstep = microstep_values[new_encoder_value];
                    break;
                }
                case PID_MENU:
                    pid_menu_index = new_encoder_value;
                    break;
                case KP_SET:
                    kp_x10 = new_encoder_value;
                    break;
                case KI_SET:
                    ki_x10 = new_encoder_value;
                    break;
                case KD_SET:
                    kd_x10 = new_encoder_value;
                    break;
                case PIN_MENU:
                    pin_menu_index = new_encoder_value;
                    break;
                case PIN_STEP_SET:
                    pin_stepper_step = new_encoder_value;
                    break;
                case PIN_DIR_SET:
                    pin_stepper_dir = new_encoder_value;
                    break;
                case PIN_ENABLE_SET:
                    pin_stepper_enable = new_encoder_value;
                    break;
                case PIN_HEATER_SET:
                    pin_heater = new_encoder_value;
                    break;
                case PIN_MAX6675_CLK_SET:
                    pin_max6675_clk = new_encoder_value;
                    break;
                case PIN_MAX6675_CS_SET:
                    pin_max6675_cs = new_encoder_value;
                    break;
                case PIN_MAX6675_DO_SET:
                    pin_max6675_do = new_encoder_value;
                    break;
                default:
                    break;
            }
        }

        // Check button click
        if (AiEsp32RotaryEncoder_isEncoderButtonClicked(&rotaryEncoder)) {
            handle_menu_click();
            needs_update = true;
        }

        // Update display if needed
        if (needs_update) {
            sh1106_clear(&display);

            switch (current_state) {
                case MAIN_MENU:
                    draw_main_menu();
                    break;
                case START_RUNNING:
                    draw_start_running();
                    break;
                case START_CONFIRM_STOP:
                    draw_confirm_stop();
                    break;
                case TEMP_SET:
                    draw_slider("SET TEMPERATURE", temperature, 30, 250, "C", icon_temp);
                    break;
                case MOTOR_SPEED_SET:
                    draw_slider("MOTOR SPEED", motor_speed, 0, 100, "%", icon_motor);
                    break;
                case MICROSTEP_SET: {
                    // Custom display for microstep selection
                    sh1106_fill_rect(&display, 0, 0, 128, 12, SH1106_COLOR_WHITE);
                    sh1106_set_text_size(&display, 1);
                    sh1106_set_text_color(&display, SH1106_COLOR_BLACK, SH1106_COLOR_WHITE);
                    sh1106_set_cursor(&display, 20, 2);
                    sh1106_print(&display, "SET MICROSTEP");
                    
                    sh1106_set_text_color(&display, SH1106_COLOR_WHITE, SH1106_COLOR_BLACK);
                    sh1106_set_text_size(&display, 2);
                    sh1106_set_cursor(&display, 40, 25);
                    char ms_str[16];
                    snprintf(ms_str, sizeof(ms_str), "1/%d", microstep);
                    sh1106_print(&display, ms_str);
                    
                    sh1106_set_text_size(&display, 1);
                    sh1106_set_cursor(&display, 20, 50);
                    sh1106_print(&display, "1  2  4  8  16 32");
                    break;
                }
                case PID_MENU:
                    draw_pid_menu();
                    break;
                case KP_SET:
                    draw_float_slider("SET Kp", kp_x10, 0, 100, "Kp");
                    break;
                case KI_SET:
                    draw_float_slider("SET Ki", ki_x10, 0, 50, "Ki");
                    break;
                case KD_SET:
                    draw_float_slider("SET Kd", kd_x10, 0, 100, "Kd");
                    break;
                case PIN_MENU:
                    draw_pin_menu();
                    break;
                case PIN_STEP_SET:
                    draw_pin_slider("STEP PIN", pin_stepper_step);
                    break;
                case PIN_DIR_SET:
                    draw_pin_slider("DIR PIN", pin_stepper_dir);
                    break;
                case PIN_ENABLE_SET:
                    draw_pin_slider("ENABLE PIN", pin_stepper_enable);
                    break;
                case PIN_HEATER_SET:
                    draw_pin_slider("HEATER PIN", pin_heater);
                    break;
                case PIN_MAX6675_CLK_SET:
                    draw_pin_slider("MAX6675 CLK", pin_max6675_clk);
                    break;
                case PIN_MAX6675_CS_SET:
                    draw_pin_slider("MAX6675 CS", pin_max6675_cs);
                    break;
                case PIN_MAX6675_DO_SET:
                    draw_pin_slider("MAX6675 DO", pin_max6675_do);
                    break;
                default:
                    break;
            }

            sh1106_display(&display);
            needs_update = false;
        }

        vTaskDelay(pdMS_TO_TICKS(5));  // Faster polling for better responsiveness
    }
}
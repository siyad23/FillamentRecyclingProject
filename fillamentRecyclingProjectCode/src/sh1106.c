#include "sh1106.h"
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "SH1106";

// 5x7 font (ASCII 32-127)
static const uint8_t font5x7[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, // Space
    0x00, 0x00, 0x5F, 0x00, 0x00, // !
    0x00, 0x07, 0x00, 0x07, 0x00, // "
    0x14, 0x7F, 0x14, 0x7F, 0x14, // #
    0x24, 0x2A, 0x7F, 0x2A, 0x12, // $
    0x23, 0x13, 0x08, 0x64, 0x62, // %
    0x36, 0x49, 0x55, 0x22, 0x50, // &
    0x00, 0x05, 0x03, 0x00, 0x00, // '
    0x00, 0x1C, 0x22, 0x41, 0x00, // (
    0x00, 0x41, 0x22, 0x1C, 0x00, // )
    0x08, 0x2A, 0x1C, 0x2A, 0x08, // *
    0x08, 0x08, 0x3E, 0x08, 0x08, // +
    0x00, 0x50, 0x30, 0x00, 0x00, // ,
    0x08, 0x08, 0x08, 0x08, 0x08, // -
    0x00, 0x60, 0x60, 0x00, 0x00, // .
    0x20, 0x10, 0x08, 0x04, 0x02, // /
    0x3E, 0x51, 0x49, 0x45, 0x3E, // 0
    0x00, 0x42, 0x7F, 0x40, 0x00, // 1
    0x42, 0x61, 0x51, 0x49, 0x46, // 2
    0x21, 0x41, 0x45, 0x4B, 0x31, // 3
    0x18, 0x14, 0x12, 0x7F, 0x10, // 4
    0x27, 0x45, 0x45, 0x45, 0x39, // 5
    0x3C, 0x4A, 0x49, 0x49, 0x30, // 6
    0x01, 0x71, 0x09, 0x05, 0x03, // 7
    0x36, 0x49, 0x49, 0x49, 0x36, // 8
    0x06, 0x49, 0x49, 0x29, 0x1E, // 9
    0x00, 0x36, 0x36, 0x00, 0x00, // :
    0x00, 0x56, 0x36, 0x00, 0x00, // ;
    0x00, 0x08, 0x14, 0x22, 0x41, // <
    0x14, 0x14, 0x14, 0x14, 0x14, // =
    0x41, 0x22, 0x14, 0x08, 0x00, // >
    0x02, 0x01, 0x51, 0x09, 0x06, // ?
    0x32, 0x49, 0x79, 0x41, 0x3E, // @
    0x7E, 0x11, 0x11, 0x11, 0x7E, // A
    0x7F, 0x49, 0x49, 0x49, 0x36, // B
    0x3E, 0x41, 0x41, 0x41, 0x22, // C
    0x7F, 0x41, 0x41, 0x22, 0x1C, // D
    0x7F, 0x49, 0x49, 0x49, 0x41, // E
    0x7F, 0x09, 0x09, 0x01, 0x01, // F
    0x3E, 0x41, 0x41, 0x51, 0x32, // G
    0x7F, 0x08, 0x08, 0x08, 0x7F, // H
    0x00, 0x41, 0x7F, 0x41, 0x00, // I
    0x20, 0x40, 0x41, 0x3F, 0x01, // J
    0x7F, 0x08, 0x14, 0x22, 0x41, // K
    0x7F, 0x40, 0x40, 0x40, 0x40, // L
    0x7F, 0x02, 0x04, 0x02, 0x7F, // M
    0x7F, 0x04, 0x08, 0x10, 0x7F, // N
    0x3E, 0x41, 0x41, 0x41, 0x3E, // O
    0x7F, 0x09, 0x09, 0x09, 0x06, // P
    0x3E, 0x41, 0x51, 0x21, 0x5E, // Q
    0x7F, 0x09, 0x19, 0x29, 0x46, // R
    0x46, 0x49, 0x49, 0x49, 0x31, // S
    0x01, 0x01, 0x7F, 0x01, 0x01, // T
    0x3F, 0x40, 0x40, 0x40, 0x3F, // U
    0x1F, 0x20, 0x40, 0x20, 0x1F, // V
    0x7F, 0x20, 0x18, 0x20, 0x7F, // W
    0x63, 0x14, 0x08, 0x14, 0x63, // X
    0x03, 0x04, 0x78, 0x04, 0x03, // Y
    0x61, 0x51, 0x49, 0x45, 0x43, // Z
    0x00, 0x00, 0x7F, 0x41, 0x41, // [
    0x02, 0x04, 0x08, 0x10, 0x20, // backslash
    0x41, 0x41, 0x7F, 0x00, 0x00, // ]
    0x04, 0x02, 0x01, 0x02, 0x04, // ^
    0x40, 0x40, 0x40, 0x40, 0x40, // _
    0x00, 0x01, 0x02, 0x04, 0x00, // `
    0x20, 0x54, 0x54, 0x54, 0x78, // a
    0x7F, 0x48, 0x44, 0x44, 0x38, // b
    0x38, 0x44, 0x44, 0x44, 0x20, // c
    0x38, 0x44, 0x44, 0x48, 0x7F, // d
    0x38, 0x54, 0x54, 0x54, 0x18, // e
    0x08, 0x7E, 0x09, 0x01, 0x02, // f
    0x08, 0x14, 0x54, 0x54, 0x3C, // g
    0x7F, 0x08, 0x04, 0x04, 0x78, // h
    0x00, 0x44, 0x7D, 0x40, 0x00, // i
    0x20, 0x40, 0x44, 0x3D, 0x00, // j
    0x00, 0x7F, 0x10, 0x28, 0x44, // k
    0x00, 0x41, 0x7F, 0x40, 0x00, // l
    0x7C, 0x04, 0x18, 0x04, 0x78, // m
    0x7C, 0x08, 0x04, 0x04, 0x78, // n
    0x38, 0x44, 0x44, 0x44, 0x38, // o
    0x7C, 0x14, 0x14, 0x14, 0x08, // p
    0x08, 0x14, 0x14, 0x18, 0x7C, // q
    0x7C, 0x08, 0x04, 0x04, 0x08, // r
    0x48, 0x54, 0x54, 0x54, 0x20, // s
    0x04, 0x3F, 0x44, 0x40, 0x20, // t
    0x3C, 0x40, 0x40, 0x20, 0x7C, // u
    0x1C, 0x20, 0x40, 0x20, 0x1C, // v
    0x3C, 0x40, 0x30, 0x40, 0x3C, // w
    0x44, 0x28, 0x10, 0x28, 0x44, // x
    0x0C, 0x50, 0x50, 0x50, 0x3C, // y
    0x44, 0x64, 0x54, 0x4C, 0x44, // z
    0x00, 0x08, 0x36, 0x41, 0x00, // {
    0x00, 0x00, 0x7F, 0x00, 0x00, // |
    0x00, 0x41, 0x36, 0x08, 0x00, // }
    0x08, 0x08, 0x2A, 0x1C, 0x08, // ~
    0x08, 0x1C, 0x2A, 0x08, 0x08, // DEL (arrow)
};

// Text state
static int16_t cursor_x = 0;
static int16_t cursor_y = 0;
static uint8_t text_size = 1;
static uint8_t text_fg = SH1106_COLOR_WHITE;
static uint8_t text_bg = SH1106_COLOR_BLACK;

static esp_err_t sh1106_write_cmd(sh1106_t *dev, uint8_t cmd) {
    uint8_t data[2] = {0x00, cmd};  // Co = 0, D/C# = 0 (command)
    return i2c_master_write_to_device(dev->i2c_port, dev->address, data, 2, pdMS_TO_TICKS(100));
}

esp_err_t sh1106_init(sh1106_t *dev, i2c_port_t port, uint8_t address) {
    dev->i2c_port = port;
    dev->address = address;
    memset(dev->buffer, 0, sizeof(dev->buffer));

    // Wait for display to power up
    vTaskDelay(pdMS_TO_TICKS(100));

    // SH1106 initialization sequence
    const uint8_t init_cmds[] = {
        0xAE,       // Display OFF
        0x02,       // Set lower column address
        0x10,       // Set higher column address
        0x40,       // Set start line address
        0xB0,       // Set page address
        0x81, 0xCF, // Set contrast control (higher value for brighter display)
        0xA1,       // Set segment re-map (column address 127 is SEG0)
        0xA6,       // Set normal display (not inverted)
        0xA8, 0x3F, // Set multiplex ratio (1 to 64)
        0xC8,       // Set COM output scan direction (remapped)
        0xD3, 0x00, // Set display offset
        0xD5, 0x80, // Set display clock divide ratio/oscillator frequency
        0xD9, 0xF1, // Set pre-charge period
        0xDA, 0x12, // Set COM pins hardware configuration
        0xDB, 0x40, // Set VCOMH deselect level
        0x8D, 0x14, // Enable charge pump (if needed)
        0xAF,       // Display ON
    };

    for (size_t i = 0; i < sizeof(init_cmds); i++) {
        esp_err_t ret = sh1106_write_cmd(dev, init_cmds[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send init command %02X at index %d", init_cmds[i], i);
            return ret;
        }
        vTaskDelay(pdMS_TO_TICKS(1));  // Small delay between commands
    }

    // Wait for display to stabilize after init
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "SH1106 initialized successfully");
    return ESP_OK;
}

void sh1106_clear(sh1106_t *dev) {
    memset(dev->buffer, 0, sizeof(dev->buffer));
}

void sh1106_display(sh1106_t *dev) {
    for (uint8_t page = 0; page < SH1106_PAGES; page++) {
        sh1106_write_cmd(dev, 0xB0 + page);  // Set page address
        sh1106_write_cmd(dev, 0x02);          // Set lower column address (offset of 2 for SH1106)
        sh1106_write_cmd(dev, 0x10);          // Set higher column address

        uint8_t data[SH1106_WIDTH + 1];
        data[0] = 0x40;  // Co = 0, D/C# = 1 (data)
        memcpy(&data[1], &dev->buffer[page * SH1106_WIDTH], SH1106_WIDTH);
        i2c_master_write_to_device(dev->i2c_port, dev->address, data, sizeof(data), pdMS_TO_TICKS(100));
    }
}

void sh1106_draw_pixel(sh1106_t *dev, int16_t x, int16_t y, uint8_t color) {
    if (x < 0 || x >= SH1106_WIDTH || y < 0 || y >= SH1106_HEIGHT) return;
    
    uint16_t idx = (y / 8) * SH1106_WIDTH + x;
    if (color) {
        dev->buffer[idx] |= (1 << (y & 7));
    } else {
        dev->buffer[idx] &= ~(1 << (y & 7));
    }
}

void sh1106_draw_line(sh1106_t *dev, int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint8_t color) {
    int16_t dx = abs(x1 - x0);
    int16_t dy = -abs(y1 - y0);
    int16_t sx = x0 < x1 ? 1 : -1;
    int16_t sy = y0 < y1 ? 1 : -1;
    int16_t err = dx + dy;

    while (1) {
        sh1106_draw_pixel(dev, x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        int16_t e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

void sh1106_draw_rect(sh1106_t *dev, int16_t x, int16_t y, int16_t w, int16_t h, uint8_t color) {
    sh1106_draw_line(dev, x, y, x + w - 1, y, color);
    sh1106_draw_line(dev, x, y + h - 1, x + w - 1, y + h - 1, color);
    sh1106_draw_line(dev, x, y, x, y + h - 1, color);
    sh1106_draw_line(dev, x + w - 1, y, x + w - 1, y + h - 1, color);
}

void sh1106_fill_rect(sh1106_t *dev, int16_t x, int16_t y, int16_t w, int16_t h, uint8_t color) {
    for (int16_t i = x; i < x + w; i++) {
        for (int16_t j = y; j < y + h; j++) {
            sh1106_draw_pixel(dev, i, j, color);
        }
    }
}

void sh1106_draw_bitmap(sh1106_t *dev, int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint8_t color) {
    int16_t byte_width = (w + 7) / 8;
    for (int16_t j = 0; j < h; j++) {
        for (int16_t i = 0; i < w; i++) {
            if (bitmap[j * byte_width + i / 8] & (0x80 >> (i & 7))) {
                sh1106_draw_pixel(dev, x + i, y + j, color);
            }
        }
    }
}

void sh1106_set_cursor(sh1106_t *dev, int16_t x, int16_t y) {
    (void)dev;
    cursor_x = x;
    cursor_y = y;
}

void sh1106_set_text_size(sh1106_t *dev, uint8_t size) {
    (void)dev;
    text_size = size > 0 ? size : 1;
}

void sh1106_set_text_color(sh1106_t *dev, uint8_t fg, uint8_t bg) {
    (void)dev;
    text_fg = fg;
    text_bg = bg;
}

void sh1106_print_char(sh1106_t *dev, char c) {
    if (c == '\n') {
        cursor_x = 0;
        cursor_y += 8 * text_size;
        return;
    }
    if (c == '\r') {
        cursor_x = 0;
        return;
    }
    if (c < 32 || c > 127) c = '?';

    for (uint8_t i = 0; i < 5; i++) {
        uint8_t line = font5x7[(c - 32) * 5 + i];
        for (uint8_t j = 0; j < 8; j++) {
            uint8_t color = (line & (1 << j)) ? text_fg : text_bg;
            if (text_size == 1) {
                sh1106_draw_pixel(dev, cursor_x + i, cursor_y + j, color);
            } else {
                sh1106_fill_rect(dev, cursor_x + i * text_size, cursor_y + j * text_size, 
                                text_size, text_size, color);
            }
        }
    }
    // Add spacing column
    for (uint8_t j = 0; j < 8; j++) {
        if (text_size == 1) {
            sh1106_draw_pixel(dev, cursor_x + 5, cursor_y + j, text_bg);
        } else {
            sh1106_fill_rect(dev, cursor_x + 5 * text_size, cursor_y + j * text_size,
                            text_size, text_size, text_bg);
        }
    }
    cursor_x += 6 * text_size;
}

void sh1106_print(sh1106_t *dev, const char *str) {
    while (*str) {
        sh1106_print_char(dev, *str++);
    }
}

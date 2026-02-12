#ifndef SH1106_H
#define SH1106_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c.h"

#define SH1106_WIDTH  128
#define SH1106_HEIGHT 64
#define SH1106_PAGES  (SH1106_HEIGHT / 8)

#define SH1106_COLOR_BLACK 0
#define SH1106_COLOR_WHITE 1

typedef struct {
    i2c_port_t i2c_port;
    uint8_t address;
    uint8_t buffer[SH1106_WIDTH * SH1106_PAGES];
} sh1106_t;

// Initialization
esp_err_t sh1106_init(sh1106_t *dev, i2c_port_t port, uint8_t address);

// Display control
void sh1106_clear(sh1106_t *dev);
void sh1106_display(sh1106_t *dev);

// Drawing primitives
void sh1106_draw_pixel(sh1106_t *dev, int16_t x, int16_t y, uint8_t color);
void sh1106_draw_line(sh1106_t *dev, int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint8_t color);
void sh1106_draw_rect(sh1106_t *dev, int16_t x, int16_t y, int16_t w, int16_t h, uint8_t color);
void sh1106_fill_rect(sh1106_t *dev, int16_t x, int16_t y, int16_t w, int16_t h, uint8_t color);
void sh1106_draw_bitmap(sh1106_t *dev, int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint8_t color);

// Text functions
void sh1106_set_cursor(sh1106_t *dev, int16_t x, int16_t y);
void sh1106_set_text_size(sh1106_t *dev, uint8_t size);
void sh1106_set_text_color(sh1106_t *dev, uint8_t fg, uint8_t bg);
void sh1106_print(sh1106_t *dev, const char *str);
void sh1106_print_char(sh1106_t *dev, char c);

#endif // SH1106_H

#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

#include <stdio.h>
#include <string.h>
#include <esp_log.h>
#include <driver/i2c.h>
#include "fonts.h"

#define DISPLAY_ADDRESS 0x3C
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64
#define DISPLAY_DIAGONAL 144 // use Pythagorean theorem c^2 = a^2 + b^2

#define I2C_SDA GPIO_NUM_5
#define I2C_SCL GPIO_NUM_6

typedef struct {
    uint8_t x;
    uint8_t y;
} cursor_coordinates;

typedef struct {
    int16_t x;
    int16_t y;
} vector2d;

void oled_display_init(void);
void oled_display_write_cmd(uint8_t cmd);
void oled_display_write_data(uint8_t* data, unsigned long length);
void oled_display_set_column_addresses(uint8_t start, uint8_t end);
void oled_display_set_page_addresses(uint8_t start, uint8_t end);
void oled_display_clear(void);
void oled_display_fill(void);
void oled_display_draw_pixel(uint8_t x, uint8_t y, uint8_t pixel_state);
void oled_display_update_buffer(void);
void oled_display_set_cursor(uint8_t x, uint8_t y);
void oled_display_write_text(char* txt, uint16_t length);
void oled_display_draw_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);
void oled_display_draw_rectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);
void oled_display_draw_triangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t x3, uint8_t y3);
uint8_t oled_display_check();

#endif // OLED_DISPLAY_H
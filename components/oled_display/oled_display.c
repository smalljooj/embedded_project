#include "oled_display.h"

static uint8_t display_buffer[(DISPLAY_HEIGHT / 8) * DISPLAY_WIDTH] = {0};
static cursor_coordinates c_coordinates = {0}; 

void oled_display_clear(void) 
{
    uint8_t clear[128] = {0};
    for (uint8_t page = 0; page < 8; page++) 
    {
        oled_display_write_data(clear, 128);
    }
}

void oled_display_draw_pixel(uint8_t x, uint8_t y, uint8_t pixel_state) 
{
    if(pixel_state)
        display_buffer[x + (DISPLAY_WIDTH * (y / 8))] |= pixel_state << (y % 8);
    else
        display_buffer[x + (DISPLAY_WIDTH * (y / 8))] &= ~(pixel_state << (y % 8));
}

void oled_display_update_buffer(void)
{
    oled_display_write_data(display_buffer, sizeof(display_buffer));
}

void oled_display_set_cursor(uint8_t x, uint8_t y)
{
    c_coordinates.x = x;
    c_coordinates.y = y;
}

void oled_display_write_text(char* txt, uint16_t length) 
{
    uint16_t char_pos;
    uint16_t next_char_pos;
    for(int i = 0; i < length; i++)
    {
        if(txt[i] == '\n') 
        {
            if (c_coordinates.y < 54)
            {
                c_coordinates.x = 0;
                c_coordinates.y += 10;
            }
        }
        else if(txt[i] >= 32 && txt[i] <= 126) 
        {
            char_pos = (txt[i] - 32) * 9;
            for(int x = 1; x < font_8x8[char_pos] + 1; x++) 
            {
                for (int y = 0; y < 8; y++)
                {
                    oled_display_draw_pixel(c_coordinates.x + x, c_coordinates.y + y, (font_8x8[char_pos + x] >> y) & 0x01);
                }
            }
            if (i < length - 1) 
            {
                next_char_pos = (txt[i + 1] - 32) * 9;
                if (c_coordinates.x + font_8x8[char_pos] + 2 < DISPLAY_WIDTH - font_8x8[next_char_pos])
                {
                    c_coordinates.x += font_8x8[char_pos] + 2;
                }
                else 
                    if (c_coordinates.y < 54)
                    {
                        c_coordinates.x = 0;
                        c_coordinates.y += 10;
                    }
            }
        }
    }
}

void oled_display_fill(void) 
{
    uint8_t clear[128] = {0xFF};
    memset(clear, 0xFF, 128);
    for (uint8_t page = 0; page < 8; page++) 
    {
        oled_display_write_data(clear, 128);
    }
}

void oled_display_write_cmd(uint8_t cmd) 
{
    i2c_cmd_handle_t link = i2c_cmd_link_create();    
    i2c_master_start(link);
    i2c_master_write_byte(link, (DISPLAY_ADDRESS << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(link, 0x00, 1);
    i2c_master_write_byte(link, cmd, 1);
    i2c_master_stop(link);
    i2c_master_cmd_begin(I2C_NUM_0, link, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(link);
}

void oled_display_write_data(uint8_t* data, unsigned long length)
{
    i2c_cmd_handle_t link = i2c_cmd_link_create();    
    i2c_master_start(link);
    i2c_master_write_byte(link, (DISPLAY_ADDRESS << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(link, 0x40, 1);
    i2c_master_write(link, data, length, 1);
    i2c_master_stop(link);
    i2c_master_cmd_begin(I2C_NUM_0, link, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(link);
}

void oled_display_set_column_addresses(uint8_t start, uint8_t end)
{
    i2c_cmd_handle_t link = i2c_cmd_link_create();    
    i2c_master_start(link);
    i2c_master_write_byte(link, (DISPLAY_ADDRESS << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(link, 0x00, 1);

    // set column address, cmd 0x21 
    i2c_master_write_byte(link, 0x21, 1);
    i2c_master_write_byte(link, start, 1);
    i2c_master_write_byte(link, end, 1);
    i2c_master_stop(link);

    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, link, pdMS_TO_TICKS(1000));
    if (ret == ESP_OK) 
        ESP_LOGI("i2c alert", "set column success");

    i2c_cmd_link_delete(link);
}

void oled_display_set_page_addresses(uint8_t start, uint8_t end)
{
    i2c_cmd_handle_t link = i2c_cmd_link_create();    
    i2c_master_start(link);
    i2c_master_write_byte(link, (DISPLAY_ADDRESS << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(link, 0x00, 1);

    // set page address, cmd 0x22 
    i2c_master_write_byte(link, 0x22, 1);
    i2c_master_write_byte(link, start, 1);
    i2c_master_write_byte(link, end, 1);
    i2c_master_stop(link);

    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, link, pdMS_TO_TICKS(1000));
    if (ret == ESP_OK) 
        ESP_LOGI("i2c alert", "set column success");

    i2c_cmd_link_delete(link);
}

void oled_display_init(void)
{
    i2c_cmd_handle_t link = i2c_cmd_link_create();    
    i2c_master_start(link);
    i2c_master_write_byte(link, (DISPLAY_ADDRESS << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(link, 0x00, 1);

    i2c_master_write_byte(link, 0xAE, 1);
    // set the oscilator frequency, cmd 0xD5 - data byte 0x80
    i2c_master_write_byte(link, 0xD5, 1);
    i2c_master_write_byte(link, 0x80, 1);
    // set mux ratio, cmd 0xA8 - data byte 0x3F ( 0x3F = 63 decimal)
    i2c_master_write_byte(link, 0xA8, 1);
    i2c_master_write_byte(link, 0x3F, 1);
    // set the display offset, cmd 0xD3 - data byte 0x00
    i2c_master_write_byte(link, 0xD3, 1);
    i2c_master_write_byte(link, 0x00, 1);
    // set the display start line, cmd 0x40 ~ 0x7F (0x7F - 0x40 =  0x3F = 63 decimal)
    i2c_master_write_byte(link, 0x40, 1);
    // enable charge pump to increase the internal voltage, and thus be able to light up the pixels
    i2c_master_write_byte(link, 0x8D, 1);
    i2c_master_write_byte(link, 0x14, 1);
    // set the address mode horizontal
    i2c_master_write_byte(link, 0x20, 1);
    i2c_master_write_byte(link, 0x00, 1);
    // set the segment re-map, 0xA0 -> col0 = seg0  -  0xA1 col127 = seg0
    i2c_master_write_byte(link, 0xA1, 1);
    // set the COM (row) scan direction, 0xC0 -> normal -  0xC8 -> remapped
    i2c_master_write_byte(link, 0xC8, 1);
    // set the hardware COM Pins configuration, cmd 0xDA - data byte 0x02 (sequential COM and disable left/right remap)
    i2c_master_write_byte(link, 0xDA, 1);
    i2c_master_write_byte(link, 0x12, 1);
    // set display constrast, cmd 0x81, data byte 0x7F (0 ~ 255, 0x7F = 127)
    i2c_master_write_byte(link, 0x81, 1);
    i2c_master_write_byte(link, 0x7F, 1);
    // resume the display, i.e. enable display outputs according to the GDDRAM contents
    i2c_master_write_byte(link, 0xA4, 1);
    // set display in normal mode
    i2c_master_write_byte(link, 0xA6, 1);

    i2c_master_stop(link);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, link, pdMS_TO_TICKS(1000));
    if (ret == ESP_OK) 
        ESP_LOGI("oled", "init success");
    i2c_cmd_link_delete(link);

    // display clear
    oled_display_clear();


    // turn on the display
    oled_display_write_cmd(0xAF);
}

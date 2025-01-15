#include <stdio.h>
#include <oled_display.h>

void i2c_init(void) {
    i2c_config_t conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = I2C_SDA,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_io_num = I2C_SCL,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = 400000,  
    };
    // configure i2c
    esp_err_t err = i2c_param_config(I2C_NUM_0, &conf);
    if (err != ESP_OK)
        ESP_LOGI("i2c error", "failed to initialize i2c");
    // install the driver
    err = i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
    if (err != ESP_OK)
        ESP_LOGI("i2c error", "failed to install the i2c driver");
}

void app_main() {
    i2c_init();

    oled_display_init();
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // select the entire display
    oled_display_set_column_addresses(0, 127);
    oled_display_set_page_addresses(0, 7);

    /*
    oled_display_draw_pixel(0, 0, 1);
    oled_display_draw_pixel(0, 2, 1);
    oled_display_draw_pixel(2, 0, 1);
    oled_display_draw_pixel(1, 1, 1);
    oled_display_draw_pixel(2, 2, 1);

    oled_display_update_buffer();
    */
   oled_display_write_text("Hello, are you \nok?", 19);
   oled_display_update_buffer();
}
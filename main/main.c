#include <stdio.h>
#include <oled_display.h>
#include <mpu.h>
#include <setjmp.h>
#include <ds18b20.h>

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


int app_main() 
{
    i2c_init();

    ds18b20_init();
    ds18b20_read_addresses();
    uint8_t address_count = ds18b20_get_address_count();
    ds18b20_adresses_print();
    float temperature;
    while(1)
    {
        for(int i = 0; i < address_count; i++)
        {
            temperature = ds18b20_read_temperature_addr(CELSIUS, i);
            printf("Temperatura: %.2f °C\n", temperature);
        }
        printf("\n");
        ds18b20_read_addresses();
        address_count = ds18b20_get_address_count();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }



    /*
    Angles angles;
    init_mpu6050();
    while (1){
        angles = calculate_angles_task();
        vTaskDelay(pdMS_TO_TICKS(500));

    }
    

    oled_display_init();
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // select the entire display
    oled_display_set_column_addresses(0, 127);
    oled_display_set_page_addresses(0, 7);

    oled_display_set_cursor(0, 0);
    oled_display_write_text("Hello, are you \nok?", 19);
    oled_display_draw_line(0, 25, 127, 25);
    oled_display_draw_rectangle(35, 35, 55, 55);
    oled_display_draw_triangle(70, 55, 100, 55, 85, 35);
    oled_display_update_buffer();
    while(1)
    {
        oled_display_check();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    */
    return 0;
}
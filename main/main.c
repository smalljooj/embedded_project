#include <stdio.h>
#include <oled_display.h>
#include <mpu.h>
#include <setjmp.h>


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


int app_main() {
    Angles angles;

    i2c_init();
    //scan_i2c_bus();

    while (true)
    {
        esp_err_t ret = init_mpu6050();
        if(ret == ESP_OK){
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(10000));
    }
    
    

    while (1){

        //mpu6050_read_gyro_x();
        angles = calculate_angles_task();
        vTaskDelay(pdMS_TO_TICKS(500));

        if ( isnan(angles.pitch) || isnan(angles.roll) ) 
        {
            while (true)
            {
                esp_err_t ret = init_mpu6050();
                if(ret == ESP_OK){
                    break;
                }

                vTaskDelay(pdMS_TO_TICKS(10000));
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(250));
    }

    //while(1){
        /*vTaskDelay(5);
        mpu6050_read_gyro_x();
        vTaskDelay(1);
        mpu6050_read_gyro_y();
        vTaskDelay(1);
        mpu6050_read_gyro_z();

        vTaskDelay(500);

        mpu6050_read_accel_x();
        vTaskDelay(1);
        mpu6050_read_accel_y();
        vTaskDelay(1);
        mpu6050_read_accel_z();
        vTaskDelay(5);*/
    //}

    return 0;
    //oled_display_init();
    //vTaskDelay(1000 / portTICK_PERIOD_MS);

    // select the entire display
    //oled_display_set_column_addresses(0, 127);
    //oled_display_set_page_addresses(0, 7);

    /*
    oled_display_draw_pixel(0, 0, 1);
    oled_display_draw_pixel(0, 2, 1);
    oled_display_draw_pixel(2, 0, 1);
    oled_display_draw_pixel(1, 1, 1);
    oled_display_draw_pixel(2, 2, 1);

    oled_display_update_buffer();
    */
   //oled_display_write_text("Hello, are you \nok?", 19);
   //oled_display_update_buffer();
}
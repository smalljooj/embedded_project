#include <stdio.h>
#include <setjmp.h>
#include <driver/ledc.h>
#include <driver/gpio.h>
#include <ds18b20.h>
#include <oled_display.h>
#include <mpu.h>
#include <pwm.h>

#define PWM_GPIO       0   // Pino onde o PWM será gerado
#define PWM_FREQ       5000 // Frequência do PWM em Hz
#define PWM_RESOLUTION LEDC_TIMER_12_BIT  // Resolução do PWM (13 bits)
#define PWM_CHANNEL    LEDC_CHANNEL_0    // Canal do PWM
#define PWM_TIMER      LEDC_TIMER_0      // Temporizador do PWM
#define BUZZER 18
#define R1 8
#define R2 1
#define R3 7
#define R4 4
#define C1 10
#define C2 11
#define C3 12
#define C4 13

uint8_t keyboard_rows[4] = {R1, R2, R3, R4};
uint8_t keyboard_cols[4] = {C1, C2, C3, C4};

void i2c_init(void);
void init(PWM* pwm);
void read_keyboard(uint8_t matriz[4][4]);

int app_main() 
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;      
    io_conf.mode = GPIO_MODE_OUTPUT;            
    io_conf.pin_bit_mask = (1ULL << BUZZER) | (1ULL << R1) | (1ULL << R2) | (1ULL << R3) | (1ULL << R4); 
    io_conf.pull_down_en = 0;                   
    io_conf.pull_up_en = 0;                     
    gpio_config(&io_conf);

    io_conf.intr_type = GPIO_INTR_DISABLE;      
    io_conf.mode = GPIO_MODE_INPUT;            
    io_conf.pin_bit_mask = (1ULL << C1) | (1ULL << C2) | (1ULL << C3) | (1ULL << C4); 
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;                   
    io_conf.pull_up_en = 0;                     
    gpio_config(&io_conf);

    //gpio_set_level(18, 1);
    //vTaskDelay(2000 / portTICK_PERIOD_MS);
    gpio_set_level(18, 0);
    PWM pwm;
    Angles angles;
    float temperature;
    init(&pwm);
    uint8_t address_count = ds18b20_get_address_count();
    uint8_t matriz[4][4];


    oled_display_write_text("Digite a senha:\n", 16);
    oled_display_write_text("    _ _ _ _", 11);
    oled_display_update_buffer();
    // menu 



    /*
    while(1) {

        read_keyboard(matriz);
        for (uint8_t i = 0; i < 4; i++)
        {
            for (uint8_t j = 0; j < 4; j++)
                printf("%d - ", matriz[i][j]);
            printf("\n");
        }
        printf("\n");
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    
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
    }

    for (int i = 0; i < 4096; i += 20)
    {
        pwm_set_duty_cycle(&pwm, i);
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    while (1){
        angles = calculate_angles_task();
        vTaskDelay(pdMS_TO_TICKS(500));

    }

    while(1)
    {
        oled_display_check();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    */
    
    return 0;
}

void read_keyboard(uint8_t matriz[4][4]) {
    gpio_set_level(R1, 0);
    gpio_set_level(R2, 0);
    gpio_set_level(R3, 0);
    gpio_set_level(R4, 0);
    for (uint8_t i = 0; i < 4; i++)
    {
        gpio_set_level(keyboard_rows[i], 1);
        for (uint8_t j = 0; j < 4; j++)
        {
           matriz[i][j] = gpio_get_level(keyboard_cols[j]);
           printf("%d - ", gpio_get_level(keyboard_cols[j]));
        }
        gpio_set_level(keyboard_rows[i], 0);
        printf("\n");
    }
    printf("\n");
    
}

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

void init(PWM *pwm) 
{
    i2c_init();
    pwm_init(&pwm, PWM_GPIO, PWM_CHANNEL, PWM_RESOLUTION, PWM_FREQ);
    ds18b20_init();
    ds18b20_read_addresses();
    init_mpu6050();
    oled_display_init();
    oled_display_set_column_addresses(0, 127);
    oled_display_set_page_addresses(0, 7);
    oled_display_set_cursor(0, 0);
}
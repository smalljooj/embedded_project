#include <stdio.h>
#include <math.h>
#include <setjmp.h>
#include <driver/ledc.h>
#include <driver/gpio.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <esp_timer.h>
#include <ds18b20.h>
#include <oled_display.h>
#include <mpu.h>
#include <pwm.h>

#define PWM_GPIO       2   // Pino onde o PWM será gerado
#define PWM_FREQ       50 // Frequência do PWM em Hz
#define PWM_RESOLUTION LEDC_TIMER_12_BIT  // Resolução do PWM (12 bits)
#define PWM_CHANNEL    LEDC_CHANNEL_2    // Canal do PWM
#define PWM_TIMER      LEDC_TIMER_0      // Temporizador do PWM
#define BUZZER 3
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

nvs_handle_t my_handle;

void i2c_init(void);
void init_modules(PWM* servo);
void read_keyboard(uint8_t matriz[4][4]);
int8_t get_keyboard_number(uint8_t matriz[4][4]);
void mpu_recalibrate();

void write_password(int32_t password);
int32_t read_password();

void open_safe_door(PWM* servo);
void close_safe_door(PWM* servo);
void beep();

float x_m = 0, y_m = 0, z_m = 0;


void show_menu_display();
void show_locked_display();
void show_password_display();
void show_change_password_display();

int8_t check_integrity()
{
    float temperature, accel_x, accel_y, accel_z;
    temperature = ds18b20_read_temperature_addr(CELSIUS, 0);
    accel_x = mpu6050_read_accel_x();
    accel_y = mpu6050_read_accel_y();
    accel_z = mpu6050_read_accel_z();

    if ((accel_x < (x_m - 0.6) || accel_x > (x_m + 0.6)) || 
        (accel_y < (y_m - 0.6) || accel_y > (y_m + 0.6)) || 
        (accel_z < (z_m - 0.6) || accel_z > (z_m + 0.6)))
    {
        return -1;
    }
    if(temperature > 40)
        return -1;
    return 0;
}

int app_main() 
{
    PWM servo;
    int64_t current_millis = 0, last_millis = 0;
    int32_t password, password_read = 0;
    uint8_t should_beep = 0, try_passord_again = 0, attempts = 0, open_menu = 0, change_pass = 0, integrity = 1;
    int8_t pass_count = 3, keyboard_number = 0,  keyboard_last_number = -1;
    uint8_t matriz[4][4];
    init_modules(&servo);

    password = read_password();
    printf("Read value: %ld\n", read_password());
    show_password_display();

    while(1) 
    {
        read_keyboard(matriz);
        if (open_menu)
        {
            keyboard_number = get_keyboard_number(matriz);
            mpu_recalibrate();
            if (keyboard_number != -1 && keyboard_number != keyboard_last_number)
            {
               if (keyboard_number == 3)
               {
                    show_change_password_display();
                    change_pass = 1;
                    open_menu = 0;
               }
               else if (keyboard_number == 7) 
               {
                    close_safe_door(&servo);
                    show_password_display();
                    open_menu = 0;
               }
            }
            keyboard_last_number = keyboard_number;
        }
        else 
        {
            if (check_integrity() == -1)
                integrity = 0;
            if (integrity || try_passord_again)
            {
                keyboard_number = get_keyboard_number(matriz);
                if (keyboard_number != -1 && keyboard_number != keyboard_last_number)
                {
                    password_read += pow(10, pass_count) * keyboard_number;
                    printf("pass: %ld\n", password_read);
                    printf("kb number: %d\n", keyboard_number);
                    oled_display_draw_rectangle(32 + 18*(3 - pass_count), 16, 40 + 18*(3 - pass_count), 28, 1);
                    oled_display_update_buffer();
                    pass_count--;
                }
                if (pass_count < 0)
                {
                    pass_count = 3;
                    if (change_pass)
                    {
                        change_pass = 0;
                        show_menu_display();
                        open_menu = 1;
                        write_password(password_read);
                        password = password_read;
                        password_read = 0;
                    }
                    else 
                    {
                        if(password_read == password)
                        {
                            open_safe_door(&servo);
                            show_menu_display();
                            should_beep = 0;
                            integrity = 1;
                            attempts = 0;
                            password_read = 0;
                            try_passord_again = 0;
                            open_menu = 1;
                        }
                        else 
                        {
                            show_password_display();
                            beep();
                            try_passord_again = 0;
                            password_read = 0;
                            attempts++;
                        }
                    }
                }
                if (attempts > 2)
                {
                    should_beep = 1;
                    integrity = 0;
                }
                keyboard_last_number = keyboard_number;
            }
            else 
            {
                show_locked_display();
                should_beep = 1;
                password_read = 0;
                if (get_keyboard_number(matriz) == 12)
                {
                    current_millis = esp_timer_get_time() / 1000;
                    if (current_millis - last_millis > 3000)
                    {
                        keyboard_last_number = get_keyboard_number(matriz);
                        try_passord_again = 1;
                        show_password_display();
                    }
                }
                else 
                {
                    last_millis = esp_timer_get_time() / 1000;
                }
            }
            if (should_beep)
                beep();
        }
        oled_display_check();
    }
    nvs_close(my_handle);
    return 0;
}

void mpu_recalibrate()
{
    for(int i = 0; i < 50; i++)
    {
        x_m += mpu6050_read_accel_x();
        y_m += mpu6050_read_accel_y();
        z_m += mpu6050_read_accel_z();
    }
    x_m /= 50;
    y_m /= 50;
    z_m /= 50;
}

void show_menu_display()
{
    oled_display_set_cursor(0, 0);
    oled_display_clear();
    oled_display_write_text("1 - change pass\n", 16);
    oled_display_write_text("2 - exit", 8);
    oled_display_update_buffer();
}

void show_locked_display()
{
    oled_display_set_cursor(0, 0);
    oled_display_clear();
    oled_display_write_text("\n\n    LOCKED!!!", 15);
    oled_display_update_buffer();
}

void show_password_display()
{
    oled_display_set_cursor(0, 0);
    oled_display_clear();
    oled_display_write_text("    Password:\n", 14);
    oled_display_write_text("    _ _ _ _", 11);
    oled_display_update_buffer();
}

void show_change_password_display()
{
    oled_display_set_cursor(0, 0);
    oled_display_clear();
    oled_display_write_text("  new Password:\n", 16);
    oled_display_write_text("    _ _ _ _", 11);
    oled_display_update_buffer();
}

void beep()
{
    gpio_set_level(BUZZER, 1);    
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(BUZZER, 0);    
}

void open_safe_door(PWM* servo)
{
    pwm_set_duty_cycle(servo, 200);
}

void close_safe_door(PWM* servo)
{
    pwm_set_duty_cycle(servo, 450);
}

void write_password(int32_t password)
{
    nvs_set_i32(my_handle, "password", password);
    nvs_commit(my_handle);
}

int32_t read_password() 
{
    int32_t password;
    nvs_get_i32(my_handle, "password", &password);
    return password;
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
        }
        gpio_set_level(keyboard_rows[i], 0);
    }
}

int8_t get_keyboard_number(uint8_t matriz[4][4]) {
    for (uint8_t i = 0; i < 4; i++)
    {
        for (uint8_t j = 0; j < 4; j++)
        {
            if (matriz[i][j])
                return i * 4 + j;
        }
    }
    return -1;
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

void init_modules(PWM *servo) 
{
    i2c_init();
    pwm_init(servo, PWM_GPIO, PWM_CHANNEL, PWM_RESOLUTION, PWM_FREQ);

    ds18b20_init();
    ds18b20_read_addresses();

    init_mpu6050();
    mpu_recalibrate();

    oled_display_init();
    oled_display_set_column_addresses(0, 127);
    oled_display_set_page_addresses(0, 7);
    oled_display_set_cursor(0, 0);

    nvs_flash_init();
    nvs_open("storage", NVS_READWRITE, &my_handle);

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
}
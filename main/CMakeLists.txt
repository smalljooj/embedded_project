idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES oled_display mpu ds18b20 pwm
                    REQUIRES pwm driver)

idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver oled_display mpu ds18b20 pwm
                    )

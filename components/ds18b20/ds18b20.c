#include <stdio.h>
#include "ds18b20.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/gpio.h"
#include <rom/ets_sys.h>
#include <esp_log.h>

uint64_t addresses[5] = {0, 0, 0, 0, 0};
int8_t branchs[64] = {0};
int8_t last_branch = 0;
int8_t conflicts;
int8_t current_conflicts;
uint8_t addr_count = 0;

void ds18b20_read_addresses()
{
    addr_count = 0;
    if (!ds18b20_restart()) {
        ESP_LOGI("ds18b20", "Fail to initiate the conversion.");
        return;
    }
    ds18b20_write_byte(0xF0);  // Search Rom

    uint64_t bit;
    uint8_t complement;
    do 
    {
        for(int i = 0; i < 64; i++)
        {
            bit = ds18b20_read_bit();
            complement = ds18b20_read_bit();

            if ((bit ^ 1) == complement)
            {
                addresses[addr_count] |= bit << i;
                ds18b20_write_bit(bit);
            }
            else 
            {
               current_conflicts++;
               if(conflicts == current_conflicts || branchs[i] == 1)
               {
                    ds18b20_write_bit(1);
                    addresses[addr_count] |= 1 << i;
                    if (branchs[i] == -1)
                    {
                        branchs[i] = 0;
                        branchs[last_branch] = -1;
                        current_conflicts--;
                    }
                    else
                    {
                        branchs[i] = 1; 
                        last_branch = i;
                    }
               }
               else 
               {
                    ds18b20_write_bit(0); 
               }
            }
        }
        if(addresses[addr_count] != addresses[addr_count - 1])
            addr_count++;
        if (conflicts == current_conflicts)
        {
            branchs[last_branch] = -1;
            current_conflicts--;
        }
        conflicts = current_conflicts;
        current_conflicts = 0;
        if (!ds18b20_restart()) {
            ESP_LOGI("ds18b20", "Fail to initiate the conversion.");
            return;
        }
        ds18b20_write_byte(0xF0);  // Search Rom
        if(conflicts < 0) conflicts = 0;
    }
    while(conflicts);
}

void ds18b20_adresses_print()
{
    printf("\n");
    for(int i = 0; i < addr_count; i++)
        printf("%d - %llx\n", i, addresses[0]);
}

uint8_t ds18b20_get_address_count(void)
{
    return addr_count;
}

// init the ds18b20 and check if it's avaliable
uint8_t ds18b20_restart(void)
{
    gpio_set_direction(DS18B20_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(DS18B20_PIN, 0);  // pull the bus down
    ets_delay_us(480);            // hold for 480us
    gpio_set_level(DS18B20_PIN, 1);  // release the bus
    gpio_set_direction(DS18B20_PIN, GPIO_MODE_INPUT);
    ets_delay_us(70);             // wait 70us

    // check if it's avaliable
    uint8_t avaliable = !gpio_get_level(DS18B20_PIN);
    ets_delay_us(410);            // wait 410us
    return avaliable;
}

void ds18b20_init(void)
{
    gpio_reset_pin(DS18B20_PIN);
    gpio_set_direction(DS18B20_PIN, GPIO_MODE_INPUT);
}

// write a bit on the bus
void ds18b20_write_bit(uint8_t bit)
{
    gpio_set_direction(DS18B20_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(DS18B20_PIN, 0);  // pull the bus down
    ets_delay_us(bit ? 5 : 60);   // hold for 5us or 60us depending on the bit value 
    gpio_set_level(DS18B20_PIN, 1);  // realease the bus
    ets_delay_us(bit ? 55 : 5);   // wait 55us or 5us;

}

// write a byte on the bus
void ds18b20_write_byte(uint8_t byte)
{
    for (int i = 0; i < 8; i++) {
        ds18b20_write_bit(byte & 0x01);  // write bit by bit
        byte >>= 1;                      // shift the next bit
    }

}

// read a bit from the bus
uint8_t ds18b20_read_bit(void)
{       
    gpio_set_direction(DS18B20_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(DS18B20_PIN, 0);  // pull the bus down
    ets_delay_us(2);              // hold for 2us
    gpio_set_direction(DS18B20_PIN, GPIO_MODE_INPUT);
    ets_delay_us(10);             // wait 10us
    uint8_t bit = gpio_get_level(DS18B20_PIN);  // read the bit
    ets_delay_us(50);             // wait 50us
    return bit;

}

// read a byte from the bus
uint8_t ds18b20_read_byte(void)
{
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        byte |= (ds18b20_read_bit() << i);  // read bit by bit from the bus
    }
    return byte;

}

// read the tempeturature
float ds18b20_read_temperature(temperature_type type) {
    if (!ds18b20_restart()) {
        ESP_LOGI("ds18b20", "Couldn't find the DS18B20.");
        return 0.0;
    }

    ds18b20_write_byte(0xCC);  // ROM ignore
    ds18b20_write_byte(0x44);  // initiate the conversion

    vTaskDelay(10 / portTICK_PERIOD_MS); // wait the end of the conversion

    if (!ds18b20_restart()) {
        ESP_LOGI("ds18b20", "Fail to initiate the conversion.");
        return 0.0;
    }

    ds18b20_write_byte(0xCC);  // ROM ignore
    ds18b20_write_byte(0xBE);  // start reading

    // read the conversion data
    uint8_t temp_lsb = ds18b20_read_byte();
    uint8_t temp_msb = ds18b20_read_byte();

    // return the temperature formatted in celsius, fahrenheit or kelvin
    int16_t raw_temp = (temp_msb << 8) | temp_lsb;
    if(type == CELSIUS)
        return (float)raw_temp / 16.0f;
    else if (type == FAHRENHEIT)
        return ((float)raw_temp / 16.0f) * 1.8 + 32; 
    else 
        return ((float)raw_temp / 16.0f) + 273.15;
}

float ds18b20_read_temperature_addr(temperature_type type, uint8_t addr_number) {
    if (!ds18b20_restart()) {
        ESP_LOGI("ds18b20", "Couldn't find the DS18B20.");
        return 0.0;
    }

    ds18b20_write_byte(0xCC);  // ROM ignore
    ds18b20_write_byte(0x44);  // initiate the conversion

    vTaskDelay(10 / portTICK_PERIOD_MS); // wait the end of the conversion

    if (!ds18b20_restart()) {
        ESP_LOGI("ds18b20", "Fail to initiate the conversion.");
        return 0.0;
    }

    ds18b20_write_byte(0x55);  // Match Rom
    for(int i = 0; i < 64; i++)
        ds18b20_write_bit((addresses[addr_number] >> i) & 0x01);
    ds18b20_write_byte(0xBE);  // start reading

    // read the conversion data
    uint8_t temp_lsb = ds18b20_read_byte();
    uint8_t temp_msb = ds18b20_read_byte();

    // return the temperature formatted in celsius, fahrenheit or kelvin
    int16_t raw_temp = (temp_msb << 8) | temp_lsb;
    if(type == CELSIUS)
        return (float)raw_temp / 16.0f;
    else if (type == FAHRENHEIT)
        return ((float)raw_temp / 16.0f) * 1.8 + 32; 
    else 
        return ((float)raw_temp / 16.0f) + 273.15;
}
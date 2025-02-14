#ifndef DS18B20_H
#define DS18B20_H

#define DS18B20_PIN 4

typedef enum {
    CELSIUS,
    FAHRENHEIT,
    KELVIN
} temperature_type;

uint8_t ds18b20_restart(void);
void ds18b20_init(void);
void ds18b20_write_bit(uint8_t bit);
void ds18b20_write_byte(uint8_t byte);
uint8_t ds18b20_read_bit(void);
uint8_t ds18b20_read_byte(void);
float ds18b20_read_temperature(temperature_type type);
void ds18b20_read_addresses();
float ds18b20_read_temperature_addr(temperature_type type, uint8_t addr_number);


#endif
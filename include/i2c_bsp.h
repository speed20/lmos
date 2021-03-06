#ifndef _I2C_BSP_H_
#define _I2C_BSP_H_
#include "stm32f4_discovery.h"

typedef struct {
	uint8_t bus_num;
	uint8_t addr;
}i2c_dev;

void i2c_init(uint8_t ch, uint32_t clock);
uint8_t i2c_write_byte(i2c_dev *dev, uint8_t reg, uint8_t value);
uint8_t i2c_write_bytes(i2c_dev *dev, uint8_t reg, uint32_t len, uint8_t *data);
uint8_t i2c_read_byte(i2c_dev *dev, uint8_t reg);
uint8_t i2c_read_bytes(i2c_dev *dev, uint8_t reg, uint32_t len, uint8_t *data);
uint8_t i2c_write_bit(i2c_dev *dev, uint8_t reg, uint8_t bit, uint8_t val);
uint8_t i2c_write_bits(i2c_dev *dev, uint8_t reg, uint8_t bit, uint8_t len, uint8_t val);
uint8_t i2c_read_bit(i2c_dev *dev, uint8_t reg, uint8_t bit);
uint8_t i2c_read_bits(i2c_dev *dev, uint8_t reg, uint8_t bit, uint8_t len);
#endif

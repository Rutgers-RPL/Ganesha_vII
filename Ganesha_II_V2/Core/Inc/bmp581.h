#ifndef BMP581_H
#define BMP581_H

#include "bmp5_defs.h"
#include "stm32h7xx_hal.h"

#include <stdint.h>

struct BMP581 {
	struct bmp5_dev device;
	struct bmp5_osr_odr_press_config odr_config;
	I2C_HandleTypeDef *hi2c;
};

BMP5_INTF_RET_TYPE read_i2c(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
BMP5_INTF_RET_TYPE write_i2c(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);
int8_t bmp581_init(struct BMP581 *bmp581, I2C_HandleTypeDef *handle);
int8_t bmp581_get_data(struct BMP581 *bmp581, struct bmp5_sensor_data *data);

#endif

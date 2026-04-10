#ifndef BMP581_H
#define BMP581_H

#include "bmp5_defs.h"
#include "stm32h7xx_hal.h"

#include <stdint.h>

#define _g_ 9.80665f

#define TROPOPAUSE_PRESSURE 22630.0f
#define STRATOSPHERE_MIDDLE_PRESSURE 5475.0f

#define STANDARD_SEA_LEVEL_PRESSURE 101325.0f
#define STANDARD_SEA_LEVEL_TEMP_K 288.15f
#define TROPOPAUSE_BASE_ALTITUDE 11000.0f
#define STRATOSPHERE_MIDDLE_BASE_ALTITUDE 20000.0f

#define TROPOSPHERE_LAPSE_RATE -0.0065f
#define AIR_MOLAR_MASS 0.0289644f
#define GAS_CONSTANT 8.31446f
#define STRATOSPHERE_BASE_TEMP_K 216.65f
#define UPPER_STRATOSPHERE_LAPSE_RATE 0.001f


struct BMP581 {
	struct bmp5_dev device;
	struct bmp5_osr_odr_press_config odr_config;
	struct bmp5_int_source_select int_config;
	I2C_HandleTypeDef *hi2c;
};

BMP5_INTF_RET_TYPE read_i2c(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
BMP5_INTF_RET_TYPE write_i2c(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);
int8_t bmp581_init(struct BMP581 *bmp581, I2C_HandleTypeDef *handle);
int8_t bmp581_update_data(struct BMP581 *bmp581, struct bmp5_sensor_data *data);
int8_t bmp581_get_power_mode(struct BMP581 *bmp581, enum bmp5_powermode *powermode);
float bmp581_estimate_altitude_msl(struct BMP581 *bmp581, struct bmp5_sensor_data *data);

#endif

#include "bmp581.h"

#include "bmp5.h"
#include "bmp5_defs.h"
#include "stm32h7xx_hal.h"

#include <stdint.h>

BMP5_INTF_RET_TYPE read_i2c(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
	I2C_HandleTypeDef *handle = (I2C_HandleTypeDef*) intf_ptr;
	HAL_StatusTypeDef read_ret = HAL_I2C_Mem_Read(handle, BMP5_I2C_ADDR_PRIM << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, length, HAL_MAX_DELAY);
	uint32_t error = HAL_I2C_GetError(handle);
	return read_ret;
}

BMP5_INTF_RET_TYPE write_i2c(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
	I2C_HandleTypeDef *handle = (I2C_HandleTypeDef*) intf_ptr;
	HAL_StatusTypeDef write_ret = HAL_I2C_Mem_Write(handle, BMP5_I2C_ADDR_PRIM << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, length, HAL_MAX_DELAY);
	return write_ret;
}

void delay(uint32_t period, void *intf_ptr) {
	HAL_Delay(period);
}

int8_t bmp581_init(struct BMP581 *bmp581, I2C_HandleTypeDef *handle)
{
	int8_t result = BMP5_OK;

	// Use I2C by default, since that is wired on our end
	bmp581->hi2c = handle;
	bmp581->device.intf = BMP5_I2C_INTF;
	bmp581->device.read = read_i2c;
	bmp581->device.write = write_i2c;
	bmp581->device.intf_ptr = handle;
	bmp581->device.delay_us = delay;
	bmp581->odr_config.odr = BMP5_ODR_240_HZ;
	bmp581->odr_config.press_en = BMP5_ENABLE;
	bmp581->int_config.drdy_en = BMP5_ENABLE;
	bmp581->int_config.fifo_full_en = BMP5_DISABLE;
	bmp581->int_config.fifo_thres_en = BMP5_DISABLE;
	bmp581->int_config.oor_press_en = BMP5_DISABLE;


	bmp5_soft_reset(&(bmp581->device));

	// Initialize the device
	result = bmp5_init(&(bmp581->device));
	if (result != BMP5_OK) {
		return result;
	}

	// Set odr frequency
	result = bmp5_set_osr_odr_press_config(&(bmp581->odr_config), &(bmp581->device));
	if (result != BMP5_OK) {
		return result;
	}

	result = bmp5_set_int_source_select(&(bmp581->int_config), &(bmp581->device));
	if (result != BMP5_OK) {
		return result;
	}
	// Enable interrupt handler
	result = bmp5_configure_interrupt(BMP5_PULSED, BMP5_ACTIVE_HIGH, BMP5_INTR_PUSH_PULL, BMP5_INTR_ENABLE, &(bmp581->device));
	if (result != BMP5_OK) {
		return result;
	}

	result = bmp5_set_power_mode(BMP5_POWERMODE_NORMAL, &(bmp581->device));
	return result;
}


int8_t bmp581_get_data(struct BMP581 *bmp581, struct bmp5_sensor_data *data)
{
	int8_t result = bmp5_get_sensor_data(data, &(bmp581->odr_config), &(bmp581->device));
	bmp581_update_altitude(bmp581, data);
	return result;
}

void bmp581_update_altitude(struct BMP581 *bmp581, struct bmp5_sensor_data *data) {
	float mbar = (data->pressure)/100.0;
	float feet = 145366.46*(1-pow((mbar/1013.25), 0.190284));
	float meters = feet * 0.3048;
	bmp581->altitude = meters;
}

int8_t bmp581_get_power_mode(struct BMP581 *bmp581, enum bmp5_powermode *powermode) {
	return bmp5_get_power_mode(powermode, &(bmp581->device));
}

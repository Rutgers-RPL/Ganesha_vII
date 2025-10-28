/*
 * bmi088.c
 *
 *  Created on: Oct 26, 2025
 *      Author: Dhruv Shah
 */

#include "bmi088.h"
#include "bmi08_defs.h"

#include <stdint.h>

BMI08_INTF_RET_TYPE bmi088_read_spi(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
	SPI_HandleTypeDef* handle = (SPI_HandleTypeDef*) intf_ptr;
	return HAL_SPI_Receive(handle, reg_data, (uint16_t)len, HAL_MAX_DELAY);
}

BMI08_INTF_RET_TYPE bmi088_write_spi(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
	SPI_HandleTypeDef* handle = (SPI_HandleTypeDef*) intf_ptr;
	return HAL_SPI_Transmit(handle, reg_data, (uint16_t)len, HAL_MAX_DELAY);
}

void bmi088_delay(uint32_t period, void *intf_ptr) {
	HAL_Delay(period*1000);
};

int8_t bmi088_init(struct bmi08_dev* bmi, SPI_HandleTypeDef* accelerometer_handle, SPI_HandleTypeDef* gyro_handle) {
	bmi->intf_ptr_accel = accelerometer_handle;
	bmi->intf_ptr_gyro = gyro_handle;
	bmi->intf = BMI08_SPI_INTF;
	bmi->variant = BMI088_VARIANT;
	bmi->dummy_byte = 0x8e;
	bmi->accel_cfg;
	bmi->gyro_cfg;
	bmi->config_file_ptr = &bmi08x_config_file;
	bmi->read_write_len = 64;
	bmi->read = bmi088_read_spi;
	bmi->write = bmi088_write_spi;
	bmi->delay_us = bmi088_delay;

	int8_t result_g = bmi08g_init(bmi);

	if (result_g != BMI08_OK) {
		return result_g;
	}

	int8_t result_a = bmi08a_init(bmi);

	if (result_a != BMI08_OK) {
		return result_a;
	}

	return 0;
}

int8_t bmi088_get_acceleration_data(struct bmi08_dev* bmi, bmi08_sensor_data* data) {
	return bmi08a_get_data(data, bmi);
};

int8_t bmi088_get_gyro_data(struct bmi08_dev* bmi, bmi08_sensor_data* data) {
	return bmi08g_get_data(data, bmi);
};

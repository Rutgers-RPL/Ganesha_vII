/*
 * bmi.c
 *
 *  Created on: Nov 21, 2025
 *      Author: Dhruv Shah
 */

#include "stm32h7xx_hal.h"
#include "bmi08x_defs.h"
#include "bmi08x.h"
#include "bmi088.h"
#include "bmi.h"

#include <stdint.h>

SPI_HandleTypeDef* hspi;

int8_t bmi_read_spi(uint8_t pin, uint8_t reg_addr, uint8_t *data, uint16_t len) {
	HAL_GPIO_WritePin(BMI088_GPIO_PORT, pin, GPIO_PIN_RESET);

	HAL_StatusTypeDef ret = HAL_OK;

	ret = HAL_SPI_Transmit(hspi, &reg_addr, 1, HAL_MAX_DELAY);

	if (ret != HAL_OK) {
		HAL_GPIO_WritePin(BMI088_GPIO_PORT, pin, GPIO_PIN_SET);
		return ret;
	}

	ret = HAL_SPI_Receive(hspi, data, len, HAL_MAX_DELAY);

	if (ret != HAL_OK) {
		HAL_GPIO_WritePin(BMI088_GPIO_PORT, pin, GPIO_PIN_SET);
		return ret;
	}

	return ret;
}

int8_t bmi_write_spi(uint8_t pin, uint8_t reg_addr, uint8_t *data, uint16_t len) {
	HAL_GPIO_WritePin(BMI088_GPIO_PORT, pin, GPIO_PIN_RESET);

	HAL_StatusTypeDef ret = HAL_OK;

	ret = HAL_SPI_Transmit(hspi, &reg_addr, 1, HAL_MAX_DELAY);

	if (ret != HAL_OK) {
		HAL_GPIO_WritePin(BMI088_GPIO_PORT, pin, GPIO_PIN_SET);
		return ret;
	}

	ret = HAL_SPI_Transmit(hspi, data, len, HAL_MAX_DELAY);

	if (ret != HAL_OK) {
		HAL_GPIO_WritePin(BMI088_GPIO_PORT, pin, GPIO_PIN_SET);
		return ret;
	}

	HAL_GPIO_WritePin(BMI088_GPIO_PORT, pin, GPIO_PIN_SET);

	return ret;
}

int8_t bmi_init(struct bmi08x_dev *dev, SPI_HandleTypeDef* spi_handle) {
	dev->accel_id = BMI088_ACCEL_PIN;
	dev->gyro_id = BMI088_GYRO_PIN;
	dev->intf = BMI08X_SPI_INTF;
	dev->read = bmi_read_spi;
	dev->write = bmi_write_spi;
	dev->delay_ms = HAL_Delay;

	hspi = spi_handle;

//	struct bmi08x_int_cfg int_config;

	int8_t ret;

	ret = bmi088_init(dev);

	if (ret != BMI08X_OK) {
		return ret;
	}

	ret = bmi08a_soft_reset(dev);

	if (ret != BMI08X_OK) {
		return ret;
	}

	dev->read_write_len = 32;
	dev->accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE;
	ret = bmi08a_set_power_mode(dev);

	if (ret != BMI08X_OK) {
		return ret;
	}

	dev->accel_cfg.range = BMI088_ACCEL_RANGE_3G;
	dev->accel_cfg.bw = BMI08X_ACCEL_BW_NORMAL;
	dev->accel_cfg.odr = BMI08X_ACCEL_ODR_200_HZ;
	ret = bmi08a_set_meas_conf(dev);

	if (ret != BMI08X_OK) {
		return ret;
	}

	dev->gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;
	bmi08g_set_power_mode(dev);

	if (ret != BMI08X_OK) {
		return ret;
	}

	dev->gyro_cfg.range = BMI08X_GYRO_RANGE_2000_DPS;
	dev->gyro_cfg.bw = BMI08X_GYRO_BW_116_ODR_1000_HZ;
	bmi08g_set_meas_conf(dev);

	if (ret != BMI08X_OK) {
		return ret;
	}

	ret = bmi088_apply_config_file(dev);

	if (ret != BMI08X_OK) {
		return ret;
	}

	return ret;
}

int8_t bmi_update_accel_data(struct bmi08x_dev *dev, struct bmi08x_sensor_data *accel_data) {
	return bmi08a_get_data(accel_data, dev);
}

int8_t bmi_update_gyro_data(struct bmi08x_dev *dev, struct bmi08x_sensor_data *gyro_data) {
	return bmi08g_get_data(gyro_data, dev);
}


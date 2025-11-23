/*
 * bmi088.c
 *
 *  Created on: Oct 26, 2025
 *      Author: Dhruv Shah
 */

#include "bmi088.h"
#include "bmi08_defs.h"
#include "bmi08.h"
#include "stm32h7xx_hal.h"
#include "bmi08x.h"

#include <stdint.h>

BMI08_INTF_RET_TYPE bmi088_read_spi(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
	struct bmi088_sensor_intf* sensor_intf = (struct bmi088_sensor_intf*) intf_ptr;

	HAL_GPIO_WritePin(BMI088_GPIO_PORT, sensor_intf->gpio_pin, GPIO_PIN_RESET);

	HAL_StatusTypeDef ret = HAL_OK;

	ret = HAL_SPI_Transmit(sensor_intf->spi_handle, &reg_addr, 1, HAL_MAX_DELAY);

	if (ret != HAL_OK) {
	    HAL_GPIO_WritePin(BMI088_GPIO_PORT, sensor_intf->gpio_pin, GPIO_PIN_SET);
		return ret;
	}

	ret = HAL_SPI_Receive(sensor_intf->spi_handle, reg_data, len, HAL_MAX_DELAY);

	if (ret != HAL_OK) {
	    HAL_GPIO_WritePin(BMI088_GPIO_PORT, sensor_intf->gpio_pin, GPIO_PIN_SET);
		return ret;
	}

    HAL_GPIO_WritePin(BMI088_GPIO_PORT, sensor_intf->gpio_pin, GPIO_PIN_SET);

	return 0;
}

BMI08_INTF_RET_TYPE bmi088_write_spi(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
	struct bmi088_sensor_intf* sensor_intf = (struct bmi088_sensor_intf*) intf_ptr;

	HAL_GPIO_WritePin(BMI088_GPIO_PORT, sensor_intf->gpio_pin, GPIO_PIN_RESET);

	HAL_StatusTypeDef ret = HAL_OK;

	ret = HAL_SPI_Transmit(sensor_intf->spi_handle, &reg_addr, 1, HAL_MAX_DELAY);

	if (ret != HAL_OK) {
		HAL_GPIO_WritePin(BMI088_GPIO_PORT, sensor_intf->gpio_pin, GPIO_PIN_SET);
		return ret;
	}

	ret = HAL_SPI_Transmit(sensor_intf->spi_handle, reg_data, len, HAL_MAX_DELAY);

	if (ret != HAL_OK) {
		HAL_GPIO_WritePin(BMI088_GPIO_PORT, sensor_intf->gpio_pin, GPIO_PIN_SET);
		return ret;
	}

    HAL_GPIO_WritePin(BMI088_GPIO_PORT, sensor_intf->gpio_pin, GPIO_PIN_SET);

    return 0;
}

void bmi088_delay(uint32_t period, void *intf_ptr) {
	HAL_Delay(period);
};

int8_t bmi088_init(struct bmi08_dev* bmi, SPI_HandleTypeDef* spi_handle) {
	struct bmi088_sensor_intf accel_intf = { BMI088_ACCEL_PIN, spi_handle };
	struct bmi088_sensor_intf gyro_intf = { BMI088_GYRO_PIN, spi_handle };

	HAL_GPIO_WritePin(BMI088_GPIO_PORT, BMI088_ACCEL_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(BMI088_GPIO_PORT, BMI088_GYRO_PIN, GPIO_PIN_SET);

	bmi->intf_ptr_accel = &accel_intf;
	bmi->intf_ptr_gyro = &gyro_intf;
	bmi->intf = BMI08_SPI_INTF;
	bmi->variant = BMI088_VARIANT;
	bmi->read_write_len = 32;
	bmi->read = bmi088_read_spi;
	bmi->write = bmi088_write_spi;
	bmi->delay_us = bmi088_delay;

	bmi->accel_cfg.power = BMI08_ACCEL_PM_ACTIVE;
	bmi->accel_cfg.range = BMI088_ACCEL_RANGE_6G;
	bmi->accel_cfg.bw = BMI08_ACCEL_BW_NORMAL;
	bmi->accel_cfg.odr = BMI08_ACCEL_ODR_200_HZ;

	bmi->gyro_cfg.power = BMI08_GYRO_PM_NORMAL;
	bmi->gyro_cfg.range = BMI08_GYRO_RANGE_1000_DPS_DPS;
	bmi->gyro_cfg.bw = BMI08_GYRO_BW_23_ODR_200_HZ;
	bmi->gyro_cfg.odr = BMI08_GYRO_BW_23_ODR_200_HZ;

	int8_t ret = BMI08_OK;

	ret = bmi08xa_init(bmi);

	if (ret != BMI08_OK) {
		return ret;
	}

	ret = bmi08a_load_config_file(bmi);

	if (ret != BMI08_OK) {
		return ret;
	}

	ret = bmi08g_init(bmi);

	if (ret != BMI08_OK) {
		return ret;
	}

	ret = bmi08a_soft_reset(bmi);

	if (ret != BMI08_OK) {
		return ret;
	}

	ret = bmi08a_set_power_mode(bmi);

	if (ret != BMI08_OK) {
		return ret;
	}

	ret = bmi08xa_set_meas_conf(bmi);

	if (ret != BMI08_OK) {
		return ret;
	}

	ret = bmi08g_soft_reset(bmi);

	if (ret != BMI08_OK) {
		return ret;
	}

	ret = bmi08g_set_power_mode(bmi);

	if (ret != BMI08_OK) {
		return ret;
	}

	ret = bmi08g_set_meas_conf(bmi);

	if (ret != BMI08_OK) {
		return ret;
	}

	return 0;
}

int8_t bmi088_update_accel_data(struct bmi08_dev* bmi, struct bmi08_sensor_data* accel_data) {
	return bmi08a_get_data(accel_data, bmi);
};

int8_t bmi088_update_gyro_data(struct bmi08_dev* bmi, struct bmi08_sensor_data* gyro_data) {
	return bmi08g_get_data(gyro_data, bmi);
};

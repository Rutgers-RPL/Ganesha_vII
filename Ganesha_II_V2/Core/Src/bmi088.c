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
	BMI08_INTF_RET_TYPE ret =  HAL_SPI_Receive(sensor_intf->spi_handle, reg_data, (uint16_t)len, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(BMI088_GPIO_PORT, sensor_intf->gpio_pin, GPIO_PIN_SET);
	return ret;
}

BMI08_INTF_RET_TYPE bmi088_write_spi(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
	struct bmi088_sensor_intf* sensor_intf = (struct bmi088_sensor_intf*) intf_ptr;
	HAL_GPIO_WritePin(BMI088_GPIO_PORT, sensor_intf->gpio_pin, GPIO_PIN_RESET);
	BMI08_INTF_RET_TYPE ret = HAL_SPI_Transmit(sensor_intf->spi_handle, reg_data, (uint16_t)len, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(BMI088_GPIO_PORT, sensor_intf->gpio_pin, GPIO_PIN_SET);
	return ret;
}

void bmi088_delay(uint32_t period, void *intf_ptr) {
	HAL_Delay(period*1000);
};

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//	if (GPIO_Pin == BMI088_ACCEL_PIN) {
//
//	}
//
//	if (GPIO_Pin == BMI088_GYRO_PIN) {
//
//	}
//}

int8_t bmi088_init(struct bmi08_dev* bmi, SPI_HandleTypeDef* spi_handle) {
	struct bmi088_sensor_intf accel_intf = { BMI088_ACCEL_PIN, spi_handle };
	struct bmi088_sensor_intf gyro_intf = { BMI088_GYRO_PIN, spi_handle };
	struct bmi08_cfg accel_config = { BMI08_ACCEL_PM_ACTIVE, (uint8_t)(6), BMI08_ACCEL_BW_NORMAL, BMI08_ACCEL_ODR_100_HZ };
	struct bmi08_cfg gyro_config = { BMI08_GYRO_PM_NORMAL, BMI08_GYRO_RANGE_250_DPS, BMI08_GYRO_BW_32_ODR_100_HZ };

	HAL_GPIO_WritePin(BMI088_GPIO_PORT, BMI088_ACCEL_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BMI088_GPIO_PORT, BMI088_GYRO_PIN, GPIO_PIN_SET);

	bmi->intf_ptr_accel = &accel_intf;
	bmi->intf_ptr_gyro = &gyro_intf;
	bmi->intf = BMI08_SPI_INTF;
	bmi->variant = BMI088_VARIANT;
	bmi->dummy_byte = 0x8e;
	bmi->accel_cfg = accel_config;
	bmi->gyro_cfg = gyro_config;
	bmi->read_write_len = 64;
	bmi->read = bmi088_read_spi;
	bmi->write = bmi088_write_spi;
	bmi->delay_us = bmi088_delay;

	int8_t result_g = bmi08g_init(bmi);

	if (result_g != BMI08_OK) {
		return result_g;
	}

	int8_t result_a = bmi08xa_init(bmi);

	if (result_a != BMI08_OK) {
		return result_a;
	}

	return 0;
}

int8_t bmi088_get_acceleration_data(struct bmi08_dev* bmi, struct bmi08_sensor_data* data) {
	return bmi08a_get_data(data, bmi);
};

int8_t bmi088_get_gyro_data(struct bmi08_dev* bmi, struct bmi08_sensor_data* data) {
	return bmi08g_get_data(data, bmi);
};

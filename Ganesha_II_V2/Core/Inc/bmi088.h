/*
 * bmi088.h
 *
 *  Created on: Oct 26, 2025
 *      Author: Dhruv Shah
 */

#ifndef INC_BMI088_H_
#define INC_BMI088_H_

#include "bmi08x.h"
#include "stm32h7xx_hal.h"
#include "stm32h743xx.h"
#include "bmi08_defs.h"

#include <stdint.h>

#define BMI088_ACCEL_PIN GPIO_PIN_3
#define BMI088_GYRO_PIN GPIO_PIN_1
#define BMI088_GPIO_PORT GPIOA

struct bmi088_sensor_intf {
	uint8_t gpio_pin;
	SPI_HandleTypeDef* spi_handle;
};

BMI08_INTF_RET_TYPE bmi088_read_spi(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
BMI08_INTF_RET_TYPE bmi088_write_spi(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
void bmi088_delay(uint32_t period, void *intf_ptr);
int8_t bmi088_init(struct bmi08_dev* bmi, SPI_HandleTypeDef* spi_handle);
int8_t bmi088_update_accel_data(struct bmi08_dev* bmi, struct bmi08_sensor_data* data);
int8_t bmi088_update_gyro_data(struct bmi08_dev* bmi, struct bmi08_sensor_data* data);

#endif /* INC_BMI088_H_ */

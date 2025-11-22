/*
 * bmi.h
 *
 *  Created on: Nov 21, 2025
 *      Author: Dhruv Shah
 */

#ifndef INC_BMI_H_
#define INC_BMI_H_

#include "bmi08x_defs.h"

#include <stdint.h>

#define BMI088_ACCEL_PIN GPIO_PIN_3
#define BMI088_GYRO_PIN GPIO_PIN_1
#define BMI088_GPIO_PORT GPIOA


int8_t bmi_init(struct bmi08x_dev *dev, SPI_HandleTypeDef* spi_handle);
int8_t bmi_update_accel_data(struct bmi08x_dev *dev, struct bmi08x_sensor_data *accel_data);
int8_t bmi_update_gyro_data(struct bmi08x_dev *dev, struct bmi08x_sensor_data *gyro_data);
int8_t bmi_read_spi(uint8_t pin, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t bmi_write_spi(uint8_t pin, uint8_t reg_addr, uint8_t *data, uint16_t len);
void bmi_delay(uint32_t period);

#endif /* INC_BMI_H_ */

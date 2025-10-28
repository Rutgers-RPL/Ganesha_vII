/*
 * bmi088.h
 *
 *  Created on: Oct 26, 2025
 *      Author: Dhruv Shah
 */

#ifndef INC_BMI088_H_
#define INC_BMI088_H_

#include "bmi08x.h"

BMI08_INTF_RET_TYPE bmi088_read_spi(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
BMI08_INTF_RET_TYPE bmi088_write_spi(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
void bmi088_delay(uint32_t period, void *intf_ptr);
int8_t bmi088_init(struct bmi08_dev* bmi, SPI_HandleTypeDef* accelerometer_handle, SPI_HandleTypeDef* gyro_handle);
int8_t bmi088_get_acceleration_data(struct bmi08_dev* bmi, bmi08_sensor_data* data);
int8_t bmi088_get_gyro_data(struct bmi08_dev* bmi, bmi08_sensor_data* data);

#endif /* INC_BMI088_H_ */

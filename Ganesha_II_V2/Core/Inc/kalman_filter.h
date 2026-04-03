/*
 * kalman_filter.h
 *
 *  Created on: Apr 3, 2026
 *      Author: Dhruv Shah
 */

#ifndef INC_KALMAN_FILTER_H_
#define INC_KALMAN_FILTER_H_

#include "arm_math.h"

struct Kalman_Filter_Instance {
	float32_t accel_quaternion[4];
	float32_t orientation_quaternion[4];
	float32_t barometer_reading;
	float32_t initial_altitude;
	float32_t altitude_estimate;
};

void kalman_filter_init(struct Kalman_Filter_Instance* instance, float32_t initial_altitude);
void kalman_filter_update(struct Kalman_Filter_Instance* instance);
void kalman_filter_update_barometer(struct Kalman_Filter_Instance* instance, float32_t reading);
void kalman_filter_update_accel(struct Kalman_Filter_Instance* instance, float32_t x, float32_t y, float32_t z);
void kalman_filter_update_orientation(struct Kalman_Filter_Instance* instance, float32_t w, float32_t x, float32_t y, float32_t z);
float32_t kalman_filter_get_altitude_estimate(struct Kalman_Filter_Instance* instance);

#endif /* INC_KALMAN_FILTER_H_ */

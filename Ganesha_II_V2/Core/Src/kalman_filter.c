/*
 * kalman_filter.c
 *
 *  Created on: Apr 3, 2026
 *      Author: Dhruv Shah
 */

#include "kalman_filter.h"
#include "arm_math.h"

void kalman_filter_init(struct Kalman_Filter_Instance* instance, float32_t initial_altitude) {
	float32_t empty_quaternion = {0.0, 0.0, 0.0, 0.0};

	memcpy(instance->accel_quaternion, empty_quaternion, sizeof(empty_quaternion));
	memcpy(instance->orientation_quaternion, empty_quaternion, sizeof(empty_quaternion));

	instance->initial_altitude = initial_altitude;
	instance->barometer_reading = 0;
}

void kalman_filter_update(struct Kalman_Filter_Instance* instance) {

}

void kalman_filter_update_barometer(struct Kalman_Filter_Instance* instance, float32_t reading) {
	instance->barometer_reading = reading;
}

void kalman_filter_update_accel(struct Kalman_Filter_Instance* instance, float32_t x, float32_t y, float32_t z) {
	instance->accel_quaternion[0] = 0.0;
	instance->accel_quaternion[1] = x;
	instance->accel_quaternion[2] = y;
	instance->accel_quaternion[3] = z;
}

void kalman_filter_update_orientation(struct Kalman_Filter_Instance* instance, float32_t w, float32_t x, float32_t y, float32_t z) {
	instance->orientation_quaternion[0] = w;
	instance->orientation_quaternion[1] = x;
	instance->orientation_quaternion[2] = y;
	instance->orientation_quaternion[3] = z;
}

float32_t kalman_filter_get_altitude_estimate(struct Kalman_Filter_Instance* instance) {
	return instance->altitude_estimate;
}

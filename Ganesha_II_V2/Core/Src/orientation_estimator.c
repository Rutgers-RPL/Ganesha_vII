/*
 * orientation_estimator.c
 *
 *  Created on: Apr 30, 2026
 *      Author: dvvsh
 */

#include "orientation_estimator.h"
#include <stdint.h>
#include "arm_math.h"
#include "us_timer.h"

void orientation_estimator_init(Orientation_Estimator* estimator) {
	estimator->quaternion[0] = 1.0f;
	estimator->quaternion[1] = 0.0f;
	estimator->quaternion[2] = 0.0f;
	estimator->quaternion[3] = 0.0f;
	estimator->prev_reading_time_us = 0;
}

void orientation_estimator_reset_from_accel(Orientation_Estimator* estimator, float accel_x, float accel_y, float accel_z) {

}

void orientation_estimator_add_gyro_reading(Orientation_Estimator* estimator, float gyro_x, float gyro_y, float gyro_z) {
	estimator->prev_reading_time_us = get_time_us();
}

float orientation_estimator_get_w(Orientation_Estimator* estimator) {
	return estimator->quaternion[0];
}

float orientation_estimator_get_x(Orientation_Estimator* estimator) {
	return estimator->quaternion[1];
}

float orientation_estimator_get_y(Orientation_Estimator* estimator) {
	return estimator->quaternion[2];
}

float orientation_estimator_get_z(Orientation_Estimator* estimator) {
	return estimator->quaternion[3];
}

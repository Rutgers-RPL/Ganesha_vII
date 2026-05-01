/*
 * orientation_estimator.h
 *
 *  Created on: Apr 30, 2026
 *      Author: Dhruv Shah
 */

#ifndef INC_ORIENTATION_ESTIMATOR_H_
#define INC_ORIENTATION_ESTIMATOR_H_

#include <stdint.h>

struct Orientation_Estimator {
	float quaternion[4];
	uint32_t prev_reading_time_us;
};

void orientation_estimator_init(Orientation_Estimator* estimator);
void orientation_estimator_reset_from_accel(Orientation_Estimator* estimator, float accel_x, float accel_y, float accel_z);
void orientation_estimator_add_gyro_reading(Orientation_Estimator* estimator, float gyro_x, float gyro_y, float gyro_z);
float orientation_estimator_get_w(Orientation_Estimator* estimator);
float orientation_estimator_get_x(Orientation_Estimator* estimator);
float orientation_estimator_get_y(Orientation_Estimator* estimator);
float orientation_estimator_get_z(Orientation_Estimator* estimator);

#endif /* INC_ORIENTATION_ESTIMATOR_H_ */

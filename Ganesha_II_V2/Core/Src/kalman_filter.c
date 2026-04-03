/*
 * kalman_filter.c
 *
 *  Created on: Apr 3, 2026
 *      Author: Dhruv Shah
 */

#include "kalman_filter.h"
#include "arm_math.h"

void kalman_filter_init(struct Kalman_Filter_Instance* instance, float32_t dt) {
    instance->dt = dt;

    instance->x_data[0] = 0.0f;
    instance->x_data[1] = 0.0f;

    instance->P_data[0] = 1.0f;
    instance->P_data[1] = 0.0f;
    instance->P_data[2] = 0.0f;
    instance->P_data[3] = 1.0f;

    instance->F_data[0] = 1.0f;
    instance->F_data[1] = dt;
    instance->F_data[2] = 0.0f;
    instance->F_data[3] = 1.0f;

    float32_t q_alt = 0.01f;
    float32_t q_vel = 0.1f;
    instance->Q_data[0] = q_alt;
    instance->Q_data[1] = 0.0f;
    instance->Q_data[2] = 0.0f;
    instance->Q_data[3] = q_vel;

    instance->H_data[0] = 1.0f;
    instance->H_data[1] = 0.0f;

    instance->R_data[0] = 0.5f;

    arm_mat_init_f32(&instance->x, 2, 1, instance->x_data);
    arm_mat_init_f32(&instance->P, 2, 2, instance->P_data);
    arm_mat_init_f32(&instance->F, 2, 2, instance->F_data);
    arm_mat_init_f32(&instance->Q, 2, 2, instance->Q_data);
    arm_mat_init_f32(&instance->H, 1, 2, instance->H_data);
    arm_mat_init_f32(&instance->R, 1, 1, instance->R_data);
    arm_mat_init_f32(&instance->K, 2, 1, instance->K_data);
    arm_mat_init_f32(&instance->y, 1, 1, instance->y_data);
    arm_mat_init_f32(&instance->S, 1, 1, instance->S_data);

    instance->barometer_reading = 0.0f;

    for (int i = 0; i < 4; i++) {
        instance->accel_quaternion[i] = 0.0f;
        instance->orientation_quaternion[i] = (i==0) ? 1.0f : 0.0f;
    }
}

void kalman_filter_update(struct Kalman_Filter_Instance* instance) {

}

void kalman_filter_update_barometric_altitude(struct Kalman_Filter_Instance* instance, float32_t barometric_altitude) {
	instance->barometric_altitude = barometric_altitude;
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
	return instance->x_data[0];
}

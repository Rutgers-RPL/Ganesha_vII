/*
 * kalman_filter.h
 *
 *  Created on: Apr 3, 2026
 *      Author: Dhruv Shah
 */

#ifndef INC_KALMAN_FILTER_H_
#define INC_KALMAN_FILTER_H_

#include "arm_math.h"

#define g_mss 9.81f

struct Kalman_Filter_Instance {
    float32_t x_data[2];
    float32_t P_data[4];
    float32_t F_data[4];
    float32_t Q_data[4];
    float32_t H_data[2];
    float32_t R_data[1];
    float32_t K_data[2];
    float32_t y_data[1];
    float32_t S_data[1];
    float32_t B_data[2];
    float32_t u_data[1];

    arm_matrix_instance_f32 x;
    arm_matrix_instance_f32 P;
    arm_matrix_instance_f32 F;
    arm_matrix_instance_f32 Q;
    arm_matrix_instance_f32 H;
    arm_matrix_instance_f32 R;
    arm_matrix_instance_f32 K;
    arm_matrix_instance_f32 y;
    arm_matrix_instance_f32 S;
    arm_matrix_instance_f32 B;
    arm_matrix_instance_f32 u;

    float32_t dt;
    float32_t barometric_altitude;
    float32_t accel_quaternion[4];
    float32_t orientation_quaternion[4];
};

void kalman_filter_init(struct Kalman_Filter_Instance* instance, float32_t dt);
void kalman_filter_update(struct Kalman_Filter_Instance* instance);
void kalman_filter_update_barometric_altitude(struct Kalman_Filter_Instance* instance, float32_t barometric_altitude);
void kalman_filter_update_accel(struct Kalman_Filter_Instance* instance, float32_t x, float32_t y, float32_t z);
void kalman_filter_update_orientation(struct Kalman_Filter_Instance* instance, float32_t w, float32_t x, float32_t y, float32_t z);
float32_t kalman_filter_get_altitude_estimate(struct Kalman_Filter_Instance* instance);

#endif /* INC_KALMAN_FILTER_H_ */

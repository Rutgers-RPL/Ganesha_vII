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

    instance->B_data[0] = 0.5f * dt * dt;
    instance->B_data[1] = dt;

    instance->u_data[0] = 0.0f;

    arm_mat_init_f32(&instance->B, 2, 1, instance->B_data);
    arm_mat_init_f32(&instance->u, 1, 1, instance->u_data);
    arm_mat_init_f32(&instance->x, 2, 1, instance->x_data);
    arm_mat_init_f32(&instance->P, 2, 2, instance->P_data);
    arm_mat_init_f32(&instance->F, 2, 2, instance->F_data);
    arm_mat_init_f32(&instance->Q, 2, 2, instance->Q_data);
    arm_mat_init_f32(&instance->H, 1, 2, instance->H_data);
    arm_mat_init_f32(&instance->R, 1, 1, instance->R_data);
    arm_mat_init_f32(&instance->K, 2, 1, instance->K_data);
    arm_mat_init_f32(&instance->y, 1, 1, instance->y_data);
    arm_mat_init_f32(&instance->S, 1, 1, instance->S_data);

    instance->barometric_altitude = 0.0f;

    for (int i = 0; i < 4; i++) {
        instance->accel_quaternion[i] = 0.0f;
        instance->orientation_quaternion[i] = (i==0) ? 1.0f : 0.0f;
    }
}

void kalman_filter_update(struct Kalman_Filter_Instance* instance) {
    // --------------------------------------------------
    // 0) NORMALIZE ORIENTATION QUATERNION
    // --------------------------------------------------
    float32_t q0 = instance->orientation_quaternion[0];
    float32_t q1 = instance->orientation_quaternion[1];
    float32_t q2 = instance->orientation_quaternion[2];
    float32_t q3 = instance->orientation_quaternion[3];

    float32_t norm = arm_sqrt_f32(q0*q0 + q1*q1 + q2*q2 + q3*q3);

    if (norm > 0.0f) {
        float32_t inv_norm = 1.0f / norm;
        instance->orientation_quaternion[0] = q0 * inv_norm;
        instance->orientation_quaternion[1] = q1 * inv_norm;
        instance->orientation_quaternion[2] = q2 * inv_norm;
        instance->orientation_quaternion[3] = q3 * inv_norm;
    }

    // --------------------------------------------------
    // 1) ROTATE ACCEL INTO WORLD FRAME
    // --------------------------------------------------
    // Compute conjugate (inverse for unit quaternion)
    float32_t orientation_conj[4] = {
        instance->orientation_quaternion[0],
       -instance->orientation_quaternion[1],
       -instance->orientation_quaternion[2],
       -instance->orientation_quaternion[3]
    };

    float32_t temp_quat[4];
    float32_t rotated_quat[4];

    // temp_quat = q ⊗ accel_quaternion
    arm_quaternion_product_single_f32(instance->orientation_quaternion, instance->accel_quaternion, temp_quat);

    // rotated_quat = (q ⊗ accel) ⊗ q_conj
    arm_quaternion_product_single_f32(temp_quat, orientation_conj, rotated_quat);

    // Extract world-frame Z and subtract gravity
    instance->u_data[0] = rotated_quat[3] - _g_;

    // --------------------------------------------------
    // 2) PREDICT STATE: x = F x + B u
    // --------------------------------------------------
    float32_t Fx_data[2], Bu_data[2];
    arm_matrix_instance_f32 Fx, Bu;

    arm_mat_init_f32(&Fx, 2, 1, Fx_data);
    arm_mat_init_f32(&Bu, 2, 1, Bu_data);

    arm_mat_mult_f32(&instance->F, &instance->x, &Fx);
    arm_mat_mult_f32(&instance->B, &instance->u, &Bu);
    arm_mat_add_f32(&Fx, &Bu, &instance->x);

    // --------------------------------------------------
    // 3) PREDICT COVARIANCE: P = F P Fᵀ + Q
    // --------------------------------------------------
    float32_t FP_data[4], FPFt_data[4], Ft_data[4];
    arm_matrix_instance_f32 FP, FPFt, Ft;

    arm_mat_init_f32(&FP, 2, 2, FP_data);
    arm_mat_init_f32(&FPFt, 2, 2, FPFt_data);
    arm_mat_init_f32(&Ft, 2, 2, Ft_data);

    arm_mat_trans_f32(&instance->F, &Ft);
    arm_mat_mult_f32(&instance->F, &instance->P, &FP);
    arm_mat_mult_f32(&FP, &Ft, &FPFt);
    arm_mat_add_f32(&FPFt, &instance->Q, &instance->P);

    // --------------------------------------------------
    // 4) MEASUREMENT UPDATE (BAROMETER)
    // --------------------------------------------------
    float32_t Hx_data[1];
    arm_matrix_instance_f32 Hx;
    arm_mat_init_f32(&Hx, 1, 1, Hx_data);

    arm_mat_mult_f32(&instance->H, &instance->x, &Hx);
    instance->y_data[0] = instance->barometric_altitude - Hx_data[0];

    // S = H P Hᵀ + R
    float32_t HP_data[2], HPHt_data[1], Ht_data[2];
    arm_matrix_instance_f32 HP, HPHt, Ht;

    arm_mat_init_f32(&HP, 1, 2, HP_data);
    arm_mat_init_f32(&HPHt, 1, 1, HPHt_data);
    arm_mat_init_f32(&Ht, 2, 1, Ht_data);

    arm_mat_trans_f32(&instance->H, &Ht);
    arm_mat_mult_f32(&instance->H, &instance->P, &HP);
    arm_mat_mult_f32(&HP, &Ht, &HPHt);
    arm_mat_add_f32(&HPHt, &instance->R, &instance->S);

    // Kalman gain K = P Hᵀ S⁻¹
    float32_t PHt_data[2];
    arm_matrix_instance_f32 PHt;
    arm_mat_init_f32(&PHt, 2, 1, PHt_data);

    arm_mat_mult_f32(&instance->P, &Ht, &PHt);

    float32_t S_inv = 1.0f / instance->S_data[0];
    instance->K_data[0] = PHt_data[0] * S_inv;
    instance->K_data[1] = PHt_data[1] * S_inv;

    // Update state: x = x + K y
    instance->x_data[0] += instance->K_data[0] * instance->y_data[0];
    instance->x_data[1] += instance->K_data[1] * instance->y_data[0];

    // --------------------------------------------------
    // 5) UPDATE COVARIANCE: P = (I - K H) P
    // --------------------------------------------------
    float32_t KH_data[4];
    float32_t I_data[4] = {1,0,0,1}, I_KH_data[4];
    arm_matrix_instance_f32 KH, I, I_KH;

    arm_mat_init_f32(&KH, 2, 2, KH_data);
    arm_mat_init_f32(&I, 2, 2, I_data);
    arm_mat_init_f32(&I_KH, 2, 2, I_KH_data);

    arm_mat_mult_f32(&instance->K, &instance->H, &KH);
    arm_mat_sub_f32(&I, &KH, &I_KH);
    arm_mat_mult_f32(&I_KH, &instance->P, &instance->P);
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

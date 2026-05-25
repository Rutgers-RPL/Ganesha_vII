/*
 * orientation_estimator.c
 *
 *  Created on: Apr 30, 2026
 *      Author: Dhruv Shah
 */

#include "orientation_estimator.h"
#include <stdint.h>
#include "us_timer.h"
#include <math.h>

void orientation_estimator_init(struct Orientation_Estimator* estimator) {
    estimator->quaternion[0] = 1.0f;
    estimator->quaternion[1] = 0.0f;
    estimator->quaternion[2] = 0.0f;
    estimator->quaternion[3] = 0.0f;
    estimator->prev_reading_time_us = get_time_us();
}

void orientation_estimator_reset_from_accel(struct Orientation_Estimator* estimator, float accel_x, float accel_y, float accel_z) {
    float norm = sqrtf(accel_x * accel_x + accel_y * accel_y +accel_z * accel_z);

    if (norm < 1e-6f) {
        return;
    }

    float accel_norm[3] = {accel_x / norm, accel_y / norm, accel_z / norm};
    float world[3] = {0.0f, 0.0f, -1.0f};

    float axis[3];
    axis[0] = accel_norm[1] * world[2] - accel_norm[2] * world[1];
    axis[1] = accel_norm[2] * world[0] - accel_norm[0] * world[2];
    axis[2] = accel_norm[0] * world[1] - accel_norm[1] * world[0];

    float dot = accel_norm[0] * world[0] + accel_norm[1] * world[1] + accel_norm[2] * world[2];

    if (dot > 1.0f) {
    	dot = 1.0f;
    }

    if (dot < -1.0f) {
    	dot = -1.0f;
    }

    float angle = acosf(dot);

    float axis_norm = sqrtf(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);

    if (axis_norm < 1e-6f) {
        if (dot > 0.0f) {
            estimator->quaternion[0] = 1.0f;
            estimator->quaternion[1] = 0.0f;
            estimator->quaternion[2] = 0.0f;
            estimator->quaternion[3] = 0.0f;
        } else {
            estimator->quaternion[0] = 0.0f;
            estimator->quaternion[1] = 1.0f;
            estimator->quaternion[2] = 0.0f;
            estimator->quaternion[3] = 0.0f;
        }
        return;
    }

    axis[0] /= axis_norm;
    axis[1] /= axis_norm;
    axis[2] /= axis_norm;

    float half_angle = 0.5f * angle;
    float sin_half = sinf(half_angle);

    estimator->quaternion[0] = cosf(half_angle);
    estimator->quaternion[1] = axis[0] * sin_half;
    estimator->quaternion[2] = axis[1] * sin_half;
    estimator->quaternion[3] = axis[2] * sin_half;
}

void orientation_estimator_add_gyro_reading(struct Orientation_Estimator* estimator, float gyro_x, float gyro_y, float gyro_z) {
    uint32_t now = get_time_us();
    float dt = (now - estimator->prev_reading_time_us) * 1e-6f;
    estimator->prev_reading_time_us = now;

    if (dt <= 0.0f || dt > 0.1f) {
        return;
    }

    float w = estimator->quaternion[0];
    float x = estimator->quaternion[1];
    float y = estimator->quaternion[2];
    float z = estimator->quaternion[3];

    float half_dt = 0.5f * dt;

    estimator->quaternion[0] += (-x * gyro_x - y * gyro_y - z * gyro_z) * half_dt;
    estimator->quaternion[1] += (w * gyro_x + y * gyro_z - z * gyro_y) * half_dt;
    estimator->quaternion[2] += (w * gyro_y - x * gyro_z + z * gyro_x) * half_dt;
    estimator->quaternion[3] += (w * gyro_z + x * gyro_y - y * gyro_x) * half_dt;

    float norm = sqrtf(
        estimator->quaternion[0] * estimator->quaternion[0] +
        estimator->quaternion[1] * estimator->quaternion[1] +
        estimator->quaternion[2] * estimator->quaternion[2] +
        estimator->quaternion[3] * estimator->quaternion[3]
    );

    if (norm > 1e-6f) {
        estimator->quaternion[0] /= norm;
        estimator->quaternion[1] /= norm;
        estimator->quaternion[2] /= norm;
        estimator->quaternion[3] /= norm;
    }
}

float orientation_estimator_get_w(struct Orientation_Estimator* estimator) {
    return estimator->quaternion[0];
}

float orientation_estimator_get_x(struct Orientation_Estimator* estimator) {
    return estimator->quaternion[1];
}

float orientation_estimator_get_y(struct Orientation_Estimator* estimator) {
    return estimator->quaternion[2];
}

float orientation_estimator_get_z(struct Orientation_Estimator* estimator) {
    return estimator->quaternion[3];
}

#ifndef CHASSIS_FUSION_H
#define CHASSIS_FUSION_H

#include "kalman_filter.h"

/* 单位：米*/
#define IMU_OFFSET_X -0.022f  // 旋转中心前方为正
#define IMU_OFFSET_Y -0.128f  // 旋转中心左方为正

#define KF_PROCESS_NOISE 1.0f // 加速度计
#define KF_MEASURE_NOISE 5.0f // 里程计

typedef struct {
    KalmanFilter_t kf_;
    float offset_x_;
    float offset_y_;
    float last_gyro_z_;
    float comp_accel_x_;
    float comp_accel_y_;
    float filtered_vx_;
    float filtered_vy_;
} ChassisFusion;

extern ChassisFusion chassis_fusion;

void initChassisFusion(ChassisFusion *fusion);
void updateChassisFusion(ChassisFusion *fusion, float odom_vx, float odom_vy, float accel_x, float accel_y, float gyro_z, float dt);

#endif // CHASSIS_FUSION_H
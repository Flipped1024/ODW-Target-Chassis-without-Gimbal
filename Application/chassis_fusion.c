#include "chassis_fusion.h"
#include <string.h>

ChassisFusion chassis_fusion;

void initChassisFusion(ChassisFusion *fusion) 
{
    fusion->offset_x_ = IMU_OFFSET_X;
    fusion->offset_y_ = IMU_OFFSET_Y;
    fusion->last_gyro_z_ = 0.0f;
    fusion->comp_accel_x_ = 0.0f;
    fusion->comp_accel_y_ = 0.0f;
    fusion->filtered_vx_ = 0.0f;
    fusion->filtered_vy_ = 0.0f;

    Kalman_Filter_Init(&fusion->kf_, 2, 2, 2);

    float p_init[4] = {10.0f, 0.0f, 0.0f, 10.0f};
    float f_init[4] = {1.0f, 0.0f, 0.0f, 1.0f};
    float b_init[4] = {0.002f, 0.0f, 0.0f, 0.002f}; 
    float h_init[4] = {1.0f, 0.0f, 0.0f, 1.0f};
    float q_init[4] = {KF_PROCESS_NOISE, 0.0f, 0.0f, KF_PROCESS_NOISE}; 
    float r_init[4] = {KF_MEASURE_NOISE, 0.0f, 0.0f, KF_MEASURE_NOISE}; 

    memcpy(fusion->kf_.P_data, p_init, sizeof(p_init));
    memcpy(fusion->kf_.F_data, f_init, sizeof(f_init));
    memcpy(fusion->kf_.B_data, b_init, sizeof(b_init));
    memcpy(fusion->kf_.H_data, h_init, sizeof(h_init));
    memcpy(fusion->kf_.Q_data, q_init, sizeof(q_init));
    memcpy(fusion->kf_.R_data, r_init, sizeof(r_init));

    fusion->kf_.UseAutoAdjustment = 0;
}

void updateChassisFusion(ChassisFusion *fusion, float odom_vx, float odom_vy, float accel_x, float accel_y, float gyro_z, float dt) 
{
    if (dt <= 0.0f) return;

    float alpha_z = (gyro_z - fusion->last_gyro_z_) / dt;
    fusion->last_gyro_z_ = gyro_z;

    fusion->comp_accel_x_ = accel_x + alpha_z * fusion->offset_y_ + gyro_z * gyro_z * fusion->offset_x_;
    fusion->comp_accel_y_ = accel_y - alpha_z * fusion->offset_x_ + gyro_z * gyro_z * fusion->offset_y_;

    fusion->kf_.ControlVector[0] = fusion->comp_accel_x_;
    fusion->kf_.ControlVector[1] = fusion->comp_accel_y_;

    fusion->kf_.MeasuredVector[0] = odom_vx;
    fusion->kf_.MeasuredVector[1] = odom_vy;

    fusion->kf_.B_data[0] = dt;
    fusion->kf_.B_data[3] = dt;

    Kalman_Filter_Update(&fusion->kf_);

    fusion->filtered_vx_ = fusion->kf_.FilteredValue[0];
    fusion->filtered_vy_ = fusion->kf_.FilteredValue[1];
}   
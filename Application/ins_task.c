#include "ins_task.h"
#include "QuaternionAHRS.h"
#include "includes.h"
#include "GravityEstimateKF.h"
#include "tim.h"
#include "bsp_buzzer.h"  

#define GYRO_ABS_LIMIT 0.5f   // 绝对角速度阈值 (rad/s)
#define GYRO_DIFF_LIMIT 0.1f // 相对波动阈值 (rad/s)
#define SETTLE_TIME_MS 500     // 剧烈晃动后的物理静置缓冲期 (1000Hz = 500次)
#define CALI_TIME_MS 2000      // 实际累加校准时间 (1000Hz = 2000次)
uint8_t is_moving = 0;

PID_t TempCtrl = {0};

uint32_t INS_DWT_Count = 0;
static float dt = 0, t = 0;
uint8_t ins_debug_mode = 0;
float RefTemp = 45.0f;

typedef enum
{
    INS_HEATING = 0, // 加热等待状态
    INS_CALIBRATING, // 零偏校准状态
    INS_NORMAL       // 正常运行状态
} INS_State_e;

INS_State_e ins_state = INS_HEATING;
extern uint8_t caliOffset;

void INS_Init(void)
{
    gEstimateKF_Init(0.01, 100000);

    PID_Init(&TempCtrl, 2000, 1200, 0, 500, 100, 0, 0, 0, 0, 0, 0,
             DerivativeFilter | Integral_Limit | Trapezoid_Intergral);
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
    
    buzzerInit(); 
}

void INS_Task(void)
{
    static uint32_t count = 0;
    dt = DWT_GetDeltaT(&INS_DWT_Count);
    t += dt;

    if ((count % 1) == 0)
    {
        buzzerUpdate(); 

        BMI088_Read(&BMI088);

        if (ins_state == INS_HEATING)
        {
            if (BMI088.Temperature > RefTemp - 0.3f)
            {
                ins_state = INS_CALIBRATING;
            }
        }
        else if (ins_state == INS_CALIBRATING)
        {
            static uint16_t settle_count = 0; // 物理静置缓冲计时器
            static uint16_t cali_count = 0;   // 校准累加计时器
            static float gyro_sum[3] = {0, 0, 0};
            static float gyro_start[3] = {0, 0, 0};

            uint8_t is_moving = 0;

            for (uint8_t i = 0; i < 3; i++)
            {
                if (fabsf(BMI088.Gyro[i]) > GYRO_ABS_LIMIT)
                {
                    is_moving = 1;
                    break;
                }
            }

            if (is_moving)
            {
                settle_count = 0;
                cali_count = 0;
                gyro_sum[0] = 0;
                gyro_sum[1] = 0;
                gyro_sum[2] = 0;
            }
            else
            {
                if (settle_count < SETTLE_TIME_MS)
                {
                    settle_count++;

                    if (settle_count == SETTLE_TIME_MS)
                    {
                        gyro_start[0] = BMI088.Gyro[0];
                        gyro_start[1] = BMI088.Gyro[1];
                        gyro_start[2] = BMI088.Gyro[2];
                    }
                }
                else
                {
                    uint8_t is_fluctuating = 0;
                    for (uint8_t i = 0; i < 3; i++)
                    {
                        if (fabsf(BMI088.Gyro[i] - gyro_start[i]) > GYRO_DIFF_LIMIT)
                        {
                            is_fluctuating = 1;
                            break;
                        }
                    }

                    if (is_fluctuating)
                    {
                        settle_count = 0;
                        cali_count = 0;
                        gyro_sum[0] = 0;
                        gyro_sum[1] = 0;
                        gyro_sum[2] = 0;
                    }
                    else
                    {
                        gyro_sum[0] += BMI088.Gyro[0];
                        gyro_sum[1] += BMI088.Gyro[1];
                        gyro_sum[2] += BMI088.Gyro[2];
                        cali_count++;

                        if (cali_count >= CALI_TIME_MS)
                        {
                            BMI088.GyroOffset[0] = gyro_sum[0] / (float)CALI_TIME_MS;
                            BMI088.GyroOffset[1] = gyro_sum[1] / (float)CALI_TIME_MS;
                            BMI088.GyroOffset[2] = gyro_sum[2] / (float)CALI_TIME_MS;

                            caliOffset = 1;

                            AHRS.q[0] = 1.0f;
                            AHRS.q[1] = 0.0f;
                            AHRS.q[2] = 0.0f;
                            AHRS.q[3] = 0.0f;
                            AHRS.Yaw = 0.0f;
                            AHRS.Pitch = 0.0f;
                            AHRS.Roll = 0.0f;

                            ins_state = INS_NORMAL; 

                            buzzerStartBeep(3); 
                        }
                    }
                }
            }
        }
        else if (ins_state == INS_NORMAL)
        {
            for (uint8_t i = 0; i < 3; i++)
            {
                AHRS.Accel[i] = BMI088.Accel[i];
                AHRS.Gyro[i] = BMI088.Gyro[i];
            }

            gEstimateKF_Update(BMI088.Gyro[X], BMI088.Gyro[Y], BMI088.Gyro[Z], AHRS.Accel[X], AHRS.Accel[Y], AHRS.Accel[Z], dt);
            Quaternion_AHRS_UpdateIMU(BMI088.Gyro[X], BMI088.Gyro[Y], BMI088.Gyro[Z], gVec[X], gVec[Y], gVec[Z], dt);
            InsertQuaternionFrame(&QuaternionBuffer, AHRS.q, USER_GetTick() / 1000.0f);

            Get_EulerAngle(AHRS.q);

            // if (GlobalDebugMode == INS_DEBUG)
            // {
            //     if (ins_debug_mode == 0)
            //         Serial_Debug(&huart1, 1, AHRS.Yaw, AHRS.Pitch, AHRS.Roll, 0, 0, 0);
            //     if (ins_debug_mode == 1)
            //         Serial_Debug(&huart1, 1, gVec[0], gVec[1], gVec[2], AHRS.Accel[X], AHRS.Accel[Y], AHRS.Accel[Z]);
            //     if (ins_debug_mode == 2)
            //         Serial_Debug(&huart1, 1, atan2f(gVec[0], gVec[2]) * RADIAN_COEF, atan2f(gVec[1], gVec[2]) * RADIAN_COEF, atan2f(AHRS.Accel[X], AHRS.Accel[Z]) * RADIAN_COEF, atan2f(AHRS.Accel[Y], AHRS.Accel[Z]) * RADIAN_COEF, 0, 0);
            // }
        }
    }

    if ((count % 2) == 0)
    {
        IMU_Temperature_Ctrl();
        // if (GlobalDebugMode == IMU_HEAT_DEBUG)
        //     Serial_Debug(&huart1, 1, RefTemp, BMI088.Temperature, TempCtrl.Output / 1000.0f, TempCtrl.Pout / 1000.0f, TempCtrl.Iout / 1000.0f, TempCtrl.Dout / 1000.0f);
    }

    count++;
}

void IMU_Temperature_Ctrl(void)
{
    PID_Calculate(&TempCtrl, BMI088.Temperature, RefTemp);

    TIM_Set_PWM(&htim10, TIM_CHANNEL_1, float_constrain(float_rounding(TempCtrl.Output), 0, UINT32_MAX));
}
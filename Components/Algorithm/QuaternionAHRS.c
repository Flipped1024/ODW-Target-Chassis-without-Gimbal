/**
  ******************************************************************************
  * @file    QuaternionAHRS.c
  * @author  Wang Hongxi
  * @version V1.1.1
  * @date    2021/5/26
  * @brief
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
#include "QuaternionAHRS.h"
#include <math.h>

AHRS_t AHRS = {0};
QuaternionBuf_t QuaternionBuffer;

volatile float twoKp = twoKpDef; // 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef; // 2 * integral gain (Ki)

volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;                 // quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f; // integral error terms scaled by Ki

static float invSqrt(float x);

/**
  * @brief          AHRS algorithm update
  * @param[1-3]     gyro measurement in rad/s
  * @param[4-6]     accelerometer measurement in m/s2
  * @param[7-9]     magnetometer measurement in uT
  * @param[10]      time stamp in s
  */
void Quaternion_AHRS_Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt)
{
    static float recipNorm;
    static float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    static float hx, hy, bx, bz;
    static float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    static float halfex, halfey, halfez;
    static float qa, qb, qc;
    // static float dt, last_time_stamp;
    // dt = (float)(time_stamp - last_time_stamp);
    // last_time_stamp = time_stamp;

    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
    {
        Quaternion_AHRS_UpdateIMU(gx, gy, gz, ax, ay, az, dt);
        return;
    }

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

        // Estimated direction of gravity and magnetic field
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        // Error is sum of cross product between estimated direction and measured direction of field vectors
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

        // Compute and apply integral feedback if enabled
        if (twoKi > 0.0f)
        {
            integralFBx += twoKi * halfex * dt; // integral error scaled by Ki
            integralFBy += twoKi * halfey * dt;
            integralFBz += twoKi * halfez * dt;
            gx += integralFBx; // apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        }
        else
        {
            integralFBx = 0.0f; // prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt); // pre-multiply common factors
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    AHRS.q[0] = q0;
    AHRS.q[1] = q1;
    AHRS.q[2] = q2;
    AHRS.q[3] = q3;
}

/**
  * @brief          IMU algorithm update
  * @param[1-3]     gyro measurement in rad/s
  * @param[4-6]     accelerometer measurement in m/s2
  * @param[7]      time stamp in s
  */
void Quaternion_AHRS_UpdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    static float recipNorm;
    static float halfvx, halfvy, halfvz;
    static float halfex, halfey, halfez;
    static float qa, qb, qc;
    // static float dt, last_time_stamp;
    // dt = (float)(time_stamp - last_time_stamp);
    // last_time_stamp = time_stamp;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Compute and apply integral feedback if enabled
        if (twoKi > 0.0f)
        {
            integralFBx += twoKi * halfex * dt; // integral error scaled by Ki
            integralFBy += twoKi * halfey * dt;
            integralFBz += twoKi * halfez * dt;
            gx += integralFBx; // apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        }
        else
        {
            integralFBx = 0.0f; // prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt); // pre-multiply common factors
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    AHRS.q[0] = q0;
    AHRS.q[1] = q1;
    AHRS.q[2] = q2;
    AHRS.q[3] = q3;
}

/**
  * @brief        Convert quaternion to eular angle
  */
void Get_EulerAngle(float *q)
{
    static int16_t Pitch_Round_Count = 0;
    static int16_t Yaw_Round_Count = 0;
    static float Pitch_Angle_Last = 0;
    static float Yaw_Angle_Last = 0;

    AHRS.Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * 57.295779513f;
    AHRS.Pitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * 57.295779513f;
    AHRS.Roll = asinf(-2.0f * (q[1] * q[3] - q[0] * q[2])) * 57.295779513f;

    // Yaw rount count
    if (AHRS.Yaw - Yaw_Angle_Last > 180.0f)
        Yaw_Round_Count--;
    else if (AHRS.Yaw - Yaw_Angle_Last < -180.0f)
        Yaw_Round_Count++;
    // Pitch rount count
    if (AHRS.Pitch - Pitch_Angle_Last > 180.0f)
        Pitch_Round_Count--;
    else if (AHRS.Pitch - Pitch_Angle_Last < -180.0f)
        Pitch_Round_Count++;

    AHRS.YawTotalAngle = 360.0f * Yaw_Round_Count + AHRS.Yaw;
    AHRS.PitchTotalAngle = 360.0f * Pitch_Round_Count + AHRS.Pitch;

    Yaw_Angle_Last = AHRS.Yaw;
    Pitch_Angle_Last = AHRS.Pitch;
}

void InsertQuaternionFrame(QuaternionBuf_t *qBuf, float *q, float time_stamp)
{
    if (qBuf->LatestNum == Q_FRAME_LEN - 1)
        qBuf->LatestNum = 0;
    else
        qBuf->LatestNum++;

    qBuf->qFrame[qBuf->LatestNum].TimeStamp = time_stamp;
    for (uint16_t i = 0; i < 4; i++)
        qBuf->qFrame[qBuf->LatestNum].q[i] = q[i];
}

uint16_t FindTimeMatchFrame(QuaternionBuf_t *qBuf, float match_time_stamp)
{
    float min_time_error = fabsf(qBuf->qFrame[0].TimeStamp - match_time_stamp);
    uint16_t num = 0;
    for (uint16_t i = 0; i < Q_FRAME_LEN; i++)
    {
        if (fabsf(qBuf->qFrame[i].TimeStamp - match_time_stamp) < min_time_error)
        {
            min_time_error = fabsf(qBuf->qFrame[i].TimeStamp - match_time_stamp);
            num = i;
        }
    }
    return num;
}

/**
  * @brief          Transform 3dvector from BodyFrame to EarthFrame
  * @param[1]       vector in BodyFrame
  * @param[2]       vector in EarthFrame
  * @param[3]       quaternion
  */
void BodyFrameToEarthFrame(float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
  * @brief          Transform 3dvector from EarthFrame to BodyFrame
  * @param[1]       vector in EarthFrame
  * @param[2]       vector in BodyFrame
  * @param[3]       quaternion
  */
void EarthFrameToBodyFrame(float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

static float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f375a86 - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

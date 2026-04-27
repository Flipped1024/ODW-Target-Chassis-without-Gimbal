#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "includes.h"

/*System constant*/
#define CHASSIS_TASK_PERIOD 2 // Period of the chasssis_task in system

/*Mechanical*/
#define WHEEL_RADIUS 76.0f
#define YAW_REDUCTION_RATIO 1.0f // Yaw_Motor's reduction ratio: 1/1
#define YAW_REDUCTION_CORRECTION_ANGLE 90.0f

#define GRAVITYCENTER_ADJUSTMENT 0.9025f

/*Control*/
#define Kx 0.25f
#define Ky 0.25f
#define VELOCITY_RATIO 6 // Spinning mode's velocity ratio
// #define MAX_RPM 12000
#define MAX_RPM 3000

/*Math calculation*/
#define user_cos arm_cos_f32
#define user_sin arm_sin_f32

/*Spinning*/
#define SPINNING_SPEED 200
#define SPINNING_OMEGA 8
#define SPINNING_B 200
#define CAP_SPINNING_OMEGA 20
#define CAP_SPINNING_B 400

/*Two heads or Four heads*/
#define NUM_OF_HEAD 2 // 2 or 4

#if ROBOT_ID == 0
#define CAP_MIN_VOLTAGE 18
#elif ROBOT_ID == 1
#define CAP_MIN_VOLTAGE 15

#endif
/*Struct of Chassis*/
typedef struct _chassis_t
{
  float Observed_Vx;
  float Observed_Vy;
  float Observed_Vr;
  float WheelRadius;
  float WheelReductionRatio;

  uint32_t Chassis_DWT_Count;
  int16_t Vx, Vy, Vr;
  float VX_k;
  float VY_k;
  float VR_k;

  float VxTransfer, VyTransfer;

  float VelocityRatio;      /*调整小陀螺在总功率中的占比*/
  float rcStickRotateRatio; /*摇杆运动与电机间的比例系数*/
  float rcMouseRotateRatio; /*鼠标运动与电机间的比例系数*/

  float V1, V2, V3, V4;

  int YawCorrectionScale;

  int8_t HeadingFlag; /*0正面 1左 -1右  2背面*/ // 步兵车头方向的转换

  float GravityCenter_Adjustment;

  float Heading;
  float Theta;
  float TotalTheta;
  int ThetaCount;
  float FollowTheta;

  float Target_Yaw;
  
  float DeflectionAngle;

  float cap_energy;
  float energy_percentage;

  float Attitude_adjustment[4];

  int8_t Spinning_direction;
  uint8_t Is_Attitude_Control_On; // 差速标志位
  uint8_t Fly_Mode;               // 飞坡模式
  uint8_t Upslope_state;  //0不助跑上坡 1助跑上坡 2下坡

  PID_t Vx_Compensate;
  PID_t Vy_Compensate;
  PID_t Vr_Compensate;

  uint8_t Mode;
  uint8_t sentry;

  Motor_t ChassisMotor[4];
  enum
  {
    FR = 0,
    FL = 1,
    HL = 2,
    HR = 3
  };

  PID_t RotateFollow;

  // float displacement_x_; 
  // float displacement_y_;
  // float traverse_target_distance_; 
  // int8_t traverse_direction_;

  float yaw_offset_; 

  float displacement_x_; 
  float displacement_y_;
  float traverse_target_distance_; 
  int8_t traverse_direction_;
} Chassis_t;
extern Chassis_t Chassis;

/*Mode enum*/
enum
{
  Follow_Mode = 0, // 跟随
  Spinning_Mode,   // 小陀螺
  Silence_Mode,    // 静止模式
  Target_Mode,
  // Auto_Traverse_Mode,
};

/*Declaration of Chassis functions*/
void Chassis_Control(void);
void Chassis_Init(void);
void Chassis_Get_Theta(void);     // Get the angle of Chassis and Gimbal
void Chassis_Set_Mode(void);      // Set Chassis's mode
void Chassis_Get_CtrlValue(void); // Handle the data from keyboard and remote control
void Chassis_Set_Control(void);   // Chassis calcutation
void Send_Chassis_Current(void);  // Send Chassis current to Motor
void Find_heading_4heads(void);   // Find heading
void Find_heading_2heads_H(void);
void Find_heading_2heads_L(void);
void Velocity_MAXLimit(void); // Max velocity limit
float Max_4(float num1, float num2, float num3, float num4);
#endif
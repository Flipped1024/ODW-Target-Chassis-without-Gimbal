#ifndef _MOTOR_H
#define _MOTOR_H

#include <stdint.h>
#include "controller.h"
#include "can.h"

#define ENCODERCOEF 0.0439453125f

#define TORQUE_CONSTANT 0.3f

#define REDUCTION_RATIO 268.0f/17.0f

//CAN Transmit ID
#define CAN_Transmit_1_4_ID 0x200
#define CAN_Transmit_5_8_ID 0x1ff
#define CAN_Transmit_6020_ID 0x2ff

//CAN Receive ID
#define CAN_Receive_1_ID 0x201
#define CAN_Receive_2_ID 0x202
#define CAN_Receive_3_ID 0x203
#define CAN_Receive_4_ID 0x204
#define CAN_Receive_5_ID 0x205
#define CAN_Receive_6_ID 0x206
#define CAN_Receive_7_ID 0x207
#define CAN_Receive_8_ID 0x208

//CAN Gimbal ID
#define CAN_GIMBAL_Info_ID 0x666
#define CAN_GIMBAL_Control_ID_1 0x233
#define CAN_GIMBAL_Control_ID_2 0x404

#define NEGATIVE 1

/*moto information receive from CAN*/
typedef __packed struct motor_t
{
  /*Read from C620*/
  uint16_t RawAngle; //abs angle range:[0,8191]
  int16_t Velocity_RPM;
  float Current;
  uint8_t Temperature;

  float last_Velocity_RPM;
  uint8_t firstRun;
  /*Data calculated*/
  uint8_t Direction;
  int32_t msg_cnt;
  int16_t Angle_offset;
  int32_t Round_cnt;
  int32_t Total_angle;
  uint16_t Last_angle;
  float Velocity_Rad;
  float Angle; //abs angle range:[0,8191]
  float AngleInDegree;
  
  float Torque;
  float Power;

  float Output;
  float Max_Out;
  float Torque_cmd;

  int16_t zero_offset;
  uint16_t CAN_ID;

  PID_t PID_Torque;
  PID_t PID_Velocity;
  PID_t PID_Angle;

  Feedforward_t FFC_Torque;
  Feedforward_t FFC_Velocity;
  Feedforward_t FFC_Angle;

  LDOB_t LDOB;

  void (*TorqueCtrl_User_Func_f)(struct motor_t *motor);
  void (*SpeedCtrl_User_Func_f)(struct motor_t *motor);
  void (*AngleCtrl_User_Func_f)(struct motor_t *motor);
	
} Motor_t;

// float Motor_Torque_Calculate(Motor_t *motor, float torque, float target_torque);
float Motor_Speed_Calculate(Motor_t *motor, float velocity, float target_speed);
float Motor_Angle_Calculate(Motor_t *motor, float angle, float velocity, float target_angle);

void get_moto_info(Motor_t *ptr, uint8_t *aData);
void get_moto_offset(Motor_t *ptr, uint8_t *aData);

HAL_StatusTypeDef Send_Motor_Current_1_4(CAN_HandleTypeDef *_hcan, int16_t c1, int16_t c2, int16_t c3, int16_t c4);
HAL_StatusTypeDef Send_Motor_Current_5_8(CAN_HandleTypeDef *_hcan, int16_t c1, int16_t c2, int16_t c3, int16_t c4);

#endif
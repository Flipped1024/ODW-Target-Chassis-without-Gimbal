#ifndef __INCLUDES_H
#define __INCLUDES_H

#define ROBOT_ID 0
// 0 - 牢五
// 1 - 牢七

// CubeMX
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

// std C
#include <stdio.h>
#include <stdint.h>

// DSP lib
#include "arm_math.h"

// user lib
#include "BMI088driver.h"
#include "BMI088reg.h"
#include "BMI088Middleware.h"

// bsp
#include "bsp_CAN.h"
#include "bsp_dwt.h"
#include "bsp_PWM.h"
#include "bsp_usart_idle.h"
#include "bsp_adc.h"
#include "bsp_i2c.h"

// application
#include "motor.h"
// #include "chassis_task.h"
// #include "client_interact.h"
#include "detect_task.h"
// #include "gimbal_task.h"
#include "ins_task.h"
#include "QuaternionAHRS.h"
#include "judgement_info.h"
#include "remote_control.h"
#include "SerialDebug.h"
// #include "VTM_info.h"
// #include "ui_task.h"
#include "power_control.h"
#include "cap.h"

#include "chassis_fusion.h"

extern uint32_t timeStamp[50];

#endif
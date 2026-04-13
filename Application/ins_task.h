#ifndef __INS_TASK_H
#define __INS_TASK_H

#include "stdint.h"
#include "includes.h"

#define INS_TASK_PERIOD 1

enum
{
  X = 0,
  Y = 1,
  Z = 2,
};

extern float RefTemp;

void INS_Init(void);
void INS_Task(void);
void IMU_Temperature_Ctrl(void);

#endif
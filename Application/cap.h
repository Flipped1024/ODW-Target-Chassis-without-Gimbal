#ifndef CAP_H
#define CAP_H
#include "includes.h"

typedef struct
{
    float Voltage;
    float Power_Chassis;
    float Power_In;
    uint8_t Cap_Normol_Chassis_Flag;
} cap_t;

void Cap_unused_correct(void);
extern cap_t Cap;
#endif
#include "power_control.h"
#include "chassis_task.h"

Chassis_PowerControl_t Power_Control;

void Chassis_Power_Cal(void)
{
    Power_Control.Power_Calculation = 0;
    for (uint8_t i = 0; i < 4; i++)
    {   
		Chassis.ChassisMotor[i].Power = fabs(Chassis.ChassisMotor[i].Torque * Chassis.ChassisMotor[i].Velocity_Rad)//Mechine Power
                                        + MOTOR_POWER_K1 * (Chassis.ChassisMotor[i].Velocity_Rad * Chassis.ChassisMotor[i].Velocity_Rad)
                                        + MOTOR_POWER_K2 * (Chassis.ChassisMotor[i].Torque * Chassis.ChassisMotor[i].Torque)
                                        + MOTOR_POWER_OFFSET;
    }
	for (uint8_t i = 0; i < 4; i++)
		Power_Control.Power_Calculation += Chassis.ChassisMotor[i].Power;
}

#ifdef DOUBLE_CAP
void Chassis_Power_Control(void)
{
    if (is_TOE_Error(CAP_TOE)) 
        Cap.Voltage = CAP_MAX_VOLTAGE; //If cap lost, set the voltage as 24V


    float Chassis_Power;//Chassis's real power
    float Chassis_Power_Buff;//Power Buff
    float Judgement_Power;//Judgement's power
    float Judgement_Power_Limit;//Power_limit received from Judgement
    float Chassis_Power_Limit;//Chassis's real power's limit

    //Set values
    Chassis_Power = Cap.Power_Chassis;//The cap is normal, read chassis's power from the cap.
	//Chassis_Power = Power_Control.Power_Calculation;
    Chassis_Power_Buff = power_heat_data.buffer_energy;//Read from the judgement

    Judgement_Power = Cap.Power_In;
    Judgement_Power_Limit = robot_state.chassis_power_limit;//read from the judgement

    //Calculate power limit
    if(Power_Control.Is_Cap_On == TRUE)
        Power_Control.Power_Limit = Judgement_Power_Limit + CAP_CURRENT*CAP_MAX_VOLTAGE;
    else
        Power_Control.Power_Limit = Judgement_Power_Limit;

    Power_Control.Power_Limit *= 0.95;
    Chassis_Power_Limit = Power_Control.Power_Limit;

    //Calculate PowerScale by the situation of power buff
    if(Chassis_Power_Buff >= MAX_POWER_BUFF)
        Power_Control.Power_Scale = 1;
    else if(Chassis_Power_Buff >= SAFE_POWER_BUFF)
        Power_Control.Power_Scale = 0.9;
    else if(Chassis_Power_Buff >= WARNING_POWER_BUFF)
    {
        Power_Control.Power_Scale = Chassis_Power_Buff/SAFE_POWER_BUFF;
        Power_Control.Power_Scale *= 0.8;
    }
    else
        Power_Control.Power_Scale = 0.05f;

    //Calculate Second_PowerScale by the situation of chassis power
    if(Power_Control.Is_Cap_On == TRUE)
    {
        if(Chassis_Power > Chassis_Power_Limit * POWER_LIMIT_SCALE_CAP)
            Power_Control.Second_Power_Scale = POWER_LIMIT_SCALE_CAP*Chassis_Power_Limit/Chassis_Power;
        else
            Power_Control.Second_Power_Scale = 1;
    }
    else 
    {
        if(Chassis_Power > Chassis_Power_Limit * POWER_LIMIT_SCALE)
            Power_Control.Second_Power_Scale = POWER_LIMIT_SCALE*Chassis_Power_Limit/Chassis_Power;
        else
            Power_Control.Second_Power_Scale = 1;
    }
		
    //Combine PowerScale and Second_PowerScale
    Power_Control.Power_Scale *= Power_Control.Second_Power_Scale;

    //The most dangerous situation
    if(Chassis_Power_Buff<=5 /*&& Judgement_Power>Judgement_Power_Limit*/)
        Power_Control.Power_Scale = 0;//Shut down

    //Put the powerscale to motors' output
	for(uint8_t i=0;i<4;i++)
	{
		Chassis.ChassisMotor[i].Output *= Power_Control.Power_Scale;
	}
}
#else
#endif

void Chassis_Power_Control_Fly(void)
{
    if (is_TOE_Error(CAP_TOE)) 
        Cap.Voltage = CAP_MAX_VOLTAGE; //If cap lost, set the voltage as 24V

    float Chassis_Power;//Chassis's real power
    float Chassis_Power_Buff;//Power Buff
    float Judgement_Power;//Judgement's power
    float Judgement_Power_Limit;//Power_limit received from Judgement
    float Chassis_Power_Limit;//Chassis's real power's limit

        //Set values
    Chassis_Power = Cap.Power_Chassis;//The cap is normal, read chassis's power from the cap.
    Chassis_Power_Buff = power_heat_data.buffer_energy;//power_heat_data.buffer_energy;//Read from the judgement

    Judgement_Power = Cap.Power_In;
    Judgement_Power_Limit = robot_state.chassis_power_limit;//robot_state.chassis_power_limit;//read from the judgement

    //Calculate power limit
    Power_Control.Power_Limit = Judgement_Power_Limit + CAP_CURRENT*CAP_MAX_VOLTAGE;
    Chassis_Power_Limit = Power_Control.Power_Limit;

    //Calculate PowerScale by the situation of power buff
    if(Chassis_Power_Buff >= MAX_POWER_BUFF_FLY)
        Power_Control.Power_Scale = 1;
    else if(Chassis_Power_Buff >= SAFE_POWER_BUFF_FLY)
        Power_Control.Power_Scale = 0.9;
    else if(Chassis_Power_Buff >= WARNING_POWER_BUFF_FLY)
    {
        Power_Control.Power_Scale = Chassis_Power_Buff/SAFE_POWER_BUFF_FLY;
        Power_Control.Power_Scale *= 0.8;
    }
    else
        Power_Control.Power_Scale = 0.05f;
    
    //Calculate Second_PowerScale by the situation of chassis power
    if(Chassis_Power > Chassis_Power_Limit *POWER_LIMIT_SCALE_CAP_FLY)
        Power_Control.Second_Power_Scale = POWER_LIMIT_SCALE_CAP_FLY*Chassis_Power_Limit/Chassis_Power;
    else
        Power_Control.Second_Power_Scale = 1;
		
    //Combine PowerScale and Second_PowerScale
    Power_Control.Power_Scale *= Power_Control.Second_Power_Scale+POWER_FLY_SCALE;

    //Put the powerscale to motors' output
	for(uint8_t i=0;i<4;i++)
	{
		Chassis.ChassisMotor[i].Output *= Power_Control.Power_Scale;
	}
}

// void Chassis_Power_Control_Without_Cap(void)
// {
//     if (is_TOE_Error(CAP_TOE)) 
//         Cap.Voltage = CAP_MAX_VOLTAGE; //If cap lost, set the voltage as 24V

// 	float Chassis_Power;//Chassis's real power
//     float Chassis_Power_Buff;//Power Buff
//     float Judgement_Power;//Judgement's power
//     float Judgement_Power_Limit;//Power_limit received from Judgement
//     float Chassis_Power_Limit;//Chassis's real power's limit

//     //Set values
//     Chassis_Power = Power_Control.Power_Calculation;//The cap is abnormal, set the calculation as the chassis's real power
//     Chassis_Power_Buff = power_heat_data.buffer_energy;//read from the judgement

//     Judgement_Power = Cap.Power_In;
//     Judgement_Power_Limit = robot_state.chassis_power_limit;//read from the judgement

//     //Calculate Power_Limit
//     Power_Control.Power_Limit = Judgement_Power_Limit;
// 	Power_Control.Power_Limit *= 0.95;
// 	Chassis_Power_Limit = Power_Control.Power_Limit;

//     //Calculate PowerScale by the situation of power buff
//     if(Chassis_Power_Buff >= MAX_POWER_BUFF)
//     {
//         Power_Control.Power_Scale = 1;
//     }
//     else if(Chassis_Power_Buff >= SAFE_POWER_BUFF)
//     {
//             Power_Control.Power_Scale = 0.8;
//     }
//     else if(Chassis_Power_Buff >= WARNING_POWER_BUFF)
//     {
//             // Power_Control.Power_Scale *= Chassis_Power_Buff/SAFE_POWER_BUFF;
//             Power_Control.Power_Scale = Chassis_Power_Buff/SAFE_POWER_BUFF;
//             Power_Control.Power_Scale *= 0.8;
//     }
//     else
//     {
//             Power_Control.Power_Scale = 0.05;
//     }

//     //Calculate Second_PowerScale by the situation of chassis power
//     if(Chassis_Power > Chassis_Power_Limit*0.18)
//         Power_Control.Second_Power_Scale = 0.18*Chassis_Power_Limit/Chassis_Power;
//     else
//         Power_Control.Second_Power_Scale = 1;
    
//     //Combine PowerScale and Second_PowerScale
//     Power_Control.Power_Scale *= Power_Control.Second_Power_Scale;

//     //The most dangerous situation
//     if(Chassis_Power_Buff==0 && Judgement_Power>Judgement_Power_Limit)
//         Power_Control.Power_Scale = 0;//Shut down

//     //Put the powerscale to motors' output
// 	for(uint8_t i=0;i<4;i++)
// 		Chassis.ChassisMotor[i].Output *= Power_Control.Power_Scale;
// }

void Chassis_Power_Control_Without_Cap(void)
{
    if (is_TOE_Error(CAP_TOE)) 
        Cap.Voltage = CAP_MAX_VOLTAGE;

    float Chassis_Power = Power_Control.Power_Calculation;
    float Chassis_Power_Buff = power_heat_data.buffer_energy;
    float Judgement_Power_Limit = robot_state.chassis_power_limit;

    if (Judgement_Power_Limit == 0) {
        Judgement_Power_Limit = 45.0f; 
    }

    // 根据裁判系统缓冲能量 (Buffer Energy) 动态缩放
    if (Chassis_Power_Buff >= 30.0f) {
        Power_Control.Power_Scale = 1.0f; 
    } else if (Chassis_Power_Buff >= 10.0f) {
        Power_Control.Power_Scale = 0.4f + ((Chassis_Power_Buff - 10.0f) / 20.0f) * 0.6f;
    } else {
        Power_Control.Power_Scale = 0.3f; 
    }

    // 根据实时功率 (Chassis Power) 动态干预
    // 在接近功率上限 (例如 90%) 时才开始干预
    if (Chassis_Power > Judgement_Power_Limit * 0.9f) {
        Power_Control.Second_Power_Scale = (Judgement_Power_Limit * 0.9f) / Chassis_Power;
    } else {
        Power_Control.Second_Power_Scale = 1.0f;
    }

    // 融合系数与防死锁兜底
    Power_Control.Power_Scale *= Power_Control.Second_Power_Scale;
    
    if (Power_Control.Power_Scale < 0.2f) {
        Power_Control.Power_Scale = 0.2f; // 全局绝对兜底，保证至少 20% 扭矩破静摩擦
    }

    //  应用限幅 
    for(uint8_t i = 0; i < 4; i++) {
        Chassis.ChassisMotor[i].PID_Velocity.MaxOut = 16384.0f * Power_Control.Power_Scale;
        Chassis.ChassisMotor[i].Output = float_constrain(Chassis.ChassisMotor[i].Output, 
                                                         -Chassis.ChassisMotor[i].PID_Velocity.MaxOut, 
                                                         Chassis.ChassisMotor[i].PID_Velocity.MaxOut);
    }
}

void Chassis_Power_Control_New(void)//In test
{
    if (is_TOE_Error(CAP_TOE)) 
        Cap.Voltage = CAP_MAX_VOLTAGE; //If cap lost, set the voltage as 24V

    //Cal Torque_cmd
    for (uint8_t i = 0; i < 4; i++)
        Chassis.ChassisMotor[i].Torque_cmd = (Chassis.ChassisMotor[i].Output * 20.0f/16384.0f) * TORQUE_CONSTANT * (1/REDUCTION_RATIO);

    /*Set values*/
    static float Judgement_Power;//Judgement's power
    static float Judgement_Power_Limit;//Power_limit received from Judgement
    static float Chassis_Power_Buff;//Power Buff

    Judgement_Power = Cap.Power_In;
    Judgement_Power_Limit = robot_state.chassis_power_limit;
    Chassis_Power_Buff = power_heat_data.buffer_energy;

    /*Calculate Power_Scale*/
    float Sum_wt;
    float Sum_wt_2;
    float Sum_t_2;
    float Sum_w_2;
    for (uint8_t i = 0; i < 4; i++)
    {
        Sum_wt += fabs(Chassis.ChassisMotor[i].Velocity_Rad * Chassis.ChassisMotor[i].Torque_cmd);
        Sum_wt_2 += (Chassis.ChassisMotor[i].Velocity_Rad * Chassis.ChassisMotor[i].Torque_cmd) * (Chassis.ChassisMotor[i].Velocity_Rad * Chassis.ChassisMotor[i].Torque_cmd);
        Sum_t_2 += Chassis.ChassisMotor[i].Torque_cmd * Chassis.ChassisMotor[i].Torque_cmd;
        Sum_w_2 += Chassis.ChassisMotor[i].Velocity_Rad * Chassis.ChassisMotor[i].Velocity_Rad;
    }

    if(Sum_t_2 != 0)
        Power_Control.Power_Scale = (-Sum_wt + sqrt(Sum_wt_2 - 4*MOTOR_POWER_K1*Sum_t_2*(MOTOR_POWER_K2*Sum_w_2-Judgement_Power_Limit)))/(2*MOTOR_POWER_K1*Sum_t_2);
    else
        Power_Control.Power_Scale = 1;

    /*Shut down*/
    if(Chassis_Power_Buff < 10 && Judgement_Power>Judgement_Power_Limit)
        Power_Control.Power_Scale = 0;

    Power_Control.Power_Scale = float_constrain(Power_Control.Power_Scale,0,1);

    //Put the powerscale to motors' output
	for(uint8_t i=0;i<4;i++)
		Chassis.ChassisMotor[i].Output *= Power_Control.Power_Scale;
}
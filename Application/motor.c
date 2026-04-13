#include "motor.h"
#include "includes.h"

// float Motor_Torque_Calculate(Motor_t *motor, float torque, float target_torque)
// {

//     Feedforward_Calculate(&motor->FFC_Torque, target_torque);

//     PID_Calculate(&motor->PID_Torque, torque, target_torque);

//     if (motor->TorqueCtrl_User_Func_f != NULL)
//         motor->TorqueCtrl_User_Func_f(motor);

//     motor->Output = motor->FFC_Torque.Output + motor->PID_Torque.Output;

//     motor->Output = float_constrain(motor->Output, -motor->Max_Out, motor->Max_Out);

//     return motor->Output;
// }

float Motor_Speed_Calculate(Motor_t *motor, float velocity, float target_speed)
{

    Feedforward_Calculate(&motor->FFC_Velocity, target_speed);

    PID_Calculate(&motor->PID_Velocity, velocity, target_speed);

    LDOB_Calculate(&motor->LDOB, velocity, motor->Output);

    if (motor->SpeedCtrl_User_Func_f != NULL)
        motor->SpeedCtrl_User_Func_f(motor);


    motor->Output = motor->FFC_Velocity.Output + motor->PID_Velocity.Output - motor->LDOB.Disturbance;

    motor->Output = float_constrain(motor->Output, -motor->Max_Out, motor->Max_Out);

    return motor->Output;
}

float Motor_Angle_Calculate(Motor_t *motor, float angle, float velocity, float target_angle)
{

    Feedforward_Calculate(&motor->FFC_Angle, target_angle);

    PID_Calculate(&motor->PID_Angle, angle, target_angle);

    if (motor->AngleCtrl_User_Func_f != NULL)
        motor->AngleCtrl_User_Func_f(motor);

    Motor_Speed_Calculate(motor, velocity, motor->FFC_Angle.Output + motor->PID_Angle.Output);

    return motor->Output;
}


void get_moto_info(Motor_t *ptr, uint8_t *aData)
{
    const float lpf = 2 * PI * 10 * 0.001 / (1 + 2 * PI * 10 * 0.001);
    /*RawAngle and Velocity_RPM*/
    if (ptr->Direction != NEGATIVE)
    {
        ptr->RawAngle = (uint16_t)(aData[0] << 8 | aData[1]);
        ptr->Velocity_RPM = (int16_t)(aData[2] << 8 | aData[3]);
    }
    else
    {
        ptr->RawAngle = 8191 - (uint16_t)(aData[0] << 8 | aData[1]);
        ptr->Velocity_RPM = -(int16_t)(aData[2] << 8 | aData[3]);
    }

    if(ptr->firstRun == 0)
    {
        ptr->firstRun = 1;
        ptr->last_Velocity_RPM = (float)(ptr->Velocity_RPM);
    }
    /*Current and Temperature*/
    ptr->Current = (int16_t)(aData[4] << 8 | aData[5]) * 20.0f/16384.0f;
    ptr->Temperature = aData[6];

    ptr->Velocity_Rad = ptr->Velocity_RPM / 9.55f; //RPM to RAD

    ptr->Velocity_RPM = lpf * (float)(ptr->Velocity_RPM) + (1.0 - lpf) * ptr->last_Velocity_RPM;
    /*Count n*/
    if (ptr->RawAngle - ptr->Last_angle > 4096)
        ptr->Round_cnt--;
    else if (ptr->RawAngle - ptr->Last_angle < -4096)
        ptr->Round_cnt++;

    /*Angle Cal*/
    ptr->Total_angle = ptr->Round_cnt * 8192 + ptr->RawAngle - ptr->Angle_offset;
    ptr->Angle = loop_float_constrain(ptr->RawAngle - ptr->zero_offset, -4095.5, 4095.5);
    ptr->AngleInDegree = ptr->Angle * 0.0439507f;

    /*Torque Cal*/
    ptr->Torque = TORQUE_CONSTANT * (1/REDUCTION_RATIO) * ptr->Current;
    /*Update*/
    ptr->Last_angle = ptr->RawAngle;
    ptr->last_Velocity_RPM = (float)(ptr->Velocity_RPM);
}


void get_moto_offset(Motor_t *ptr, uint8_t *aData)
{
    ptr->RawAngle = (uint16_t)(aData[0] << 8 | aData[1]);
    ptr->Angle_offset = ptr->RawAngle;
    ptr->Last_angle = ptr->RawAngle; 
    ptr->msg_cnt ++;
}

HAL_StatusTypeDef Send_Motor_Current_1_4(CAN_HandleTypeDef *_hcan,
                                         int16_t c1, int16_t c2, int16_t c3, int16_t c4)
{
    static CAN_TxHeaderTypeDef TX_MSG;
    static uint8_t CAN_Send_Data[8];
    uint32_t send_mail_box;

   
    TX_MSG.StdId = CAN_Transmit_1_4_ID;
    TX_MSG.IDE = CAN_ID_STD;
    TX_MSG.RTR = CAN_RTR_DATA;
    TX_MSG.DLC = 0x08;

    CAN_Send_Data[0] = (c1 >> 8);
    CAN_Send_Data[1] = c1;
    CAN_Send_Data[2] = (c2 >> 8);
    CAN_Send_Data[3] = c2;
    CAN_Send_Data[4] = (c3 >> 8);
    CAN_Send_Data[5] = c3;
    CAN_Send_Data[6] = (c4 >> 8);
    CAN_Send_Data[7] = c4;

    while (!((_hcan->State == HAL_CAN_STATE_READY) || (_hcan->State == HAL_CAN_STATE_LISTENING)))
    {
    }
    while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0) 
    {
    }
    /* Check Tx Mailbox 0 status */
    if ((_hcan->Instance->TSR & CAN_TSR_TME0) != 0U)
    {
        send_mail_box = CAN_TX_MAILBOX0;
    }
    /* Check Tx Mailbox 1 status */
    else if ((_hcan->Instance->TSR & CAN_TSR_TME1) != 0U)
    {
        send_mail_box = CAN_TX_MAILBOX1;
    }

    /* Check Tx Mailbox 2 status */
    else if ((_hcan->Instance->TSR & CAN_TSR_TME2) != 0U)
    {
        send_mail_box = CAN_TX_MAILBOX2;
    }
    return HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
}

HAL_StatusTypeDef Send_Motor_Current_5_8(CAN_HandleTypeDef *_hcan,
                                         int16_t c1, int16_t c2, int16_t c3, int16_t c4)
{
    static CAN_TxHeaderTypeDef TX_MSG;
    static uint8_t CAN_Send_Data[8];
    uint32_t send_mail_box;

    TX_MSG.StdId = CAN_Transmit_5_8_ID;
    TX_MSG.IDE = CAN_ID_STD;
    TX_MSG.RTR = CAN_RTR_DATA;
    TX_MSG.DLC = 0x08;
    CAN_Send_Data[0] = (c1 >> 8);
    CAN_Send_Data[1] = c1;
    CAN_Send_Data[2] = (c2 >> 8);
    CAN_Send_Data[3] = c2;
    CAN_Send_Data[4] = (c3 >> 8);
    CAN_Send_Data[5] = c3;
    CAN_Send_Data[6] = (c4 >> 8);
    CAN_Send_Data[7] = c4;

    while (!((_hcan->State == HAL_CAN_STATE_READY) || (_hcan->State == HAL_CAN_STATE_LISTENING)))
    {
    }
    while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0) 
    {
    }
    /* Check Tx Mailbox 0 status */
    if ((_hcan->Instance->TSR & CAN_TSR_TME0) != 0U)
    {
        send_mail_box = CAN_TX_MAILBOX0;
    }
    /* Check Tx Mailbox 1 status */
    else if ((_hcan->Instance->TSR & CAN_TSR_TME1) != 0U)
    {
        send_mail_box = CAN_TX_MAILBOX1;
    }

    /* Check Tx Mailbox 2 status */
    else if ((_hcan->Instance->TSR & CAN_TSR_TME2) != 0U)
    {
        send_mail_box = CAN_TX_MAILBOX2;
    }
    return HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
}
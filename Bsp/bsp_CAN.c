#include "bsp_CAN.h"
// #include "VTM_info.h"
#include "bsp_dwt.h"
#include "chassis_task.h"

int16_t count_CAN[6] = {0, 0, 0, 0, 0, 0};

void CAN_Device_Init(void)
{
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;

    while (HAL_CAN_ConfigFilter(&hcan1, &can_filter_st) != HAL_OK)
    {
    }

    while (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
    }

    while (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
    }

    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;

    while (HAL_CAN_ConfigFilter(&hcan2, &can_filter_st) != HAL_OK)
    {
    }

    while (HAL_CAN_Start(&hcan2) != HAL_OK)
    {
    }

    while (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *_hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    static uint8_t RC_Data_Buf[16];
    static uint8_t Gimbal_Date_Buf[8];
    // static int16_t CAP_vol;

    HAL_CAN_GetRxMessage(_hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    if (_hcan == &hcan1)
    {
        if (rx_header.StdId >= 0x201 && rx_header.StdId <= 0x204)
        {
            if (Chassis.ChassisMotor[rx_header.StdId - 0x201].msg_cnt <= 50)
            {
                get_moto_offset(&Chassis.ChassisMotor[rx_header.StdId - 0x201], rx_data);
            }

            else
            {
                get_moto_info(&Chassis.ChassisMotor[rx_header.StdId - 0x201], rx_data);
                count_CAN[0]++;
            }
            switch (rx_header.StdId)
            {
            case 0x201:
            {
                Detect_Hook(CHASSIS_MOTOR1_TOE);
                break;
            }
            case 0x202:
            {
                Detect_Hook(CHASSIS_MOTOR2_TOE);
                break;
            }
            case 0x203:
            {
                Detect_Hook(CHASSIS_MOTOR3_TOE);
                break;
            }
            case 0x204:
            {
                Detect_Hook(CHASSIS_MOTOR4_TOE);
                break;
            }
            }
        }
        else
        {
            switch (rx_header.StdId)
            {

            // case CAN_RC_DATA_Frame_0:
            //     RC_Data_Buf[0] = rx_data[0];
            //     RC_Data_Buf[1] = rx_data[1];
            //     RC_Data_Buf[2] = rx_data[2];
            //     RC_Data_Buf[3] = rx_data[3];
            //     RC_Data_Buf[4] = rx_data[4];
            //     RC_Data_Buf[5] = rx_data[5];
            //     RC_Data_Buf[6] = rx_data[6];
            //     RC_Data_Buf[7] = rx_data[7];
            //     break;
            // case CAN_RC_DATA_Frame_1:
            //     RC_Data_Buf[8] = rx_data[0];
            //     RC_Data_Buf[9] = rx_data[1];
            //     RC_Data_Buf[10] = rx_data[2];
            //     RC_Data_Buf[11] = rx_data[3];
            //     RC_Data_Buf[12] = rx_data[4];
            //     RC_Data_Buf[13] = rx_data[5];
            //     RC_Data_Buf[14] = rx_data[6];
            //     RC_Data_Buf[15] = rx_data[7];
            //     Callback_RC_Handle(&remote_control, RC_Data_Buf);
            //     break;

            // case CAN_CAP_INFO_ID:
            //     CAP_vol = rx_data[0];
            //     Cap.Voltage = (float)CAP_vol / 5.0f;
            //     Cap.Power_Chassis = (float)((rx_data[1] << 8 | rx_data[2]) / 10.0f);
            //     Cap.Power_In = (float)((rx_data[3] << 8 | rx_data[4]) / 10.0f);
            //     Cap.Cap_Normol_Chassis_Flag = rx_data[5];
            //     Detect_Hook(CAP_TOE);
            //     break;
            }
        }
    }
    if (_hcan == &hcan2)
    {
        switch (rx_header.StdId)
        {

        case CAN_RC_DATA_Frame_0:
            RC_Data_Buf[0] = rx_data[0];
            RC_Data_Buf[1] = rx_data[1];
            RC_Data_Buf[2] = rx_data[2];
            RC_Data_Buf[3] = rx_data[3];
            RC_Data_Buf[4] = rx_data[4];
            RC_Data_Buf[5] = rx_data[5];
            RC_Data_Buf[6] = rx_data[6];
            RC_Data_Buf[7] = rx_data[7];
            break;
        case CAN_RC_DATA_Frame_1:
            RC_Data_Buf[8] = rx_data[0];
            RC_Data_Buf[9] = rx_data[1];
            RC_Data_Buf[10] = rx_data[2];
            RC_Data_Buf[11] = rx_data[3];
            RC_Data_Buf[12] = rx_data[4];
            RC_Data_Buf[13] = rx_data[5];
            RC_Data_Buf[14] = rx_data[6];
            RC_Data_Buf[15] = rx_data[7];
            count_CAN[2]++;
            Callback_RC_Handle(&remote_control, RC_Data_Buf);
            break;
        // case CAN_VTM_DATA_Frame_0:
        //     VTM_Data_Buf[0] = rx_data[0];
        //     VTM_Data_Buf[1] = rx_data[1];
        //     VTM_Data_Buf[2] = rx_data[2];
        //     VTM_Data_Buf[3] = rx_data[3];
        //     VTM_Data_Buf[4] = rx_data[4];
        //     VTM_Data_Buf[5] = rx_data[5];
        //     VTM_Data_Buf[6] = rx_data[6];
        //     VTM_Data_Buf[7] = rx_data[7];
        //     break;
        // case CAN_VTM_DATA_Frame_1:
        //     VTM_Data_Buf[8] = rx_data[0];
        //     VTM_Data_Buf[9] = rx_data[1];
        //     VTM_Data_Buf[10] = rx_data[2];
        //     VTM_Data_Buf[11] = rx_data[3];
        //     memcpy(&robot_command, VTM_Data_Buf, 12);
        //     Detect_Hook(VTM_TOE);
        //     if (is_TOE_Error(RC_TOE))
        //     {
        //         remote_control.switch_left = Switch_Middle;
        //         remote_control.switch_right = Switch_Middle;
        //         remote_control.mouse.x = robot_command.mouse_x;
        //         remote_control.mouse.y = robot_command.mouse_y;
        //         remote_control.mouse.z = robot_command.mouse_z;
        //         remote_control.mouse.press_right = robot_command.right_button_down;
        //         remote_control.mouse.press_left = robot_command.left_button_down;
        //         remote_control.key_code = robot_command.keyboard_value;
        //     }
        //     break;

        case CAN_SYSTEM_RESET_CMD:
            HAL_NVIC_SystemReset();
            break;
        // case YAW_MOTOR_ID:

        //     if (Gimbal.YawMotor.msg_cnt <= 50)
        //         get_moto_offset(&Gimbal.YawMotor, rx_data);
        //     else
        //         get_moto_info(&Gimbal.YawMotor, rx_data);
        //     Detect_Hook(GIMBAL_YAW_MOTOR_TOE);
        //     count_CAN[3]++;
        //     break;
        // case PITCH_MOTOR_ID:
        //     if (Gimbal.PitchMotor.msg_cnt <= 50)
        //         get_moto_offset(&Gimbal.PitchMotor, rx_data);
        //     else
        //         get_moto_info(&Gimbal.PitchMotor, rx_data);
        //     Detect_Hook(GIMBAL_PITCH_MOTOR_TOE);
        //     break;
        // case GIMBAL_DATE_ID:
        //     Gimbal_Date_Buf[0] = rx_data[0];
        //     Gimbal_Date_Buf[1] = rx_data[1];
        //     Gimbal_Date_Buf[2] = rx_data[2];
        //     Gimbal_Date_Buf[3] = rx_data[3];
        //     Gimbal_Date_Buf[4] = rx_data[4];
        //     Gimbal_Date_Buf[5] = rx_data[5];
        //     Gimbal_Date_Buf[6] = rx_data[6];
        //     Gimbal_Date_Buf[7] = rx_data[7];
        //     Callback_Gimbal_Handle(&Gimbal_Date, Gimbal_Date_Buf);
        //     break;
        }
    }
}

void Send_RC_Data(CAN_HandleTypeDef *_hcan, uint8_t *rc_data)
{
    static CAN_TxHeaderTypeDef TX_MSG;
    static uint8_t CAN_Send_Data[8];
    uint32_t send_mail_box;

    TX_MSG.StdId = CAN_RC_DATA_Frame_0;
    TX_MSG.IDE = CAN_ID_STD;
    TX_MSG.RTR = CAN_RTR_DATA;
    TX_MSG.DLC = 0x08;
    CAN_Send_Data[0] = rc_data[0];
    CAN_Send_Data[1] = rc_data[1];
    CAN_Send_Data[2] = rc_data[2];
    CAN_Send_Data[3] = rc_data[3];
    CAN_Send_Data[4] = rc_data[4];
    CAN_Send_Data[5] = rc_data[5];
    CAN_Send_Data[6] = rc_data[6];
    CAN_Send_Data[7] = rc_data[7];

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
    HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);

    TX_MSG.StdId = CAN_RC_DATA_Frame_1;
    CAN_Send_Data[0] = rc_data[8];
    CAN_Send_Data[1] = rc_data[9];
    CAN_Send_Data[2] = rc_data[10];
    CAN_Send_Data[3] = rc_data[11];
    CAN_Send_Data[4] = rc_data[12];
    CAN_Send_Data[5] = rc_data[13];
    CAN_Send_Data[6] = rc_data[14];
    CAN_Send_Data[7] = rc_data[15];

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
    HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
}

void Send_VTM_Data(CAN_HandleTypeDef *_hcan, uint8_t *vtm_data)
{
    static CAN_TxHeaderTypeDef TX_MSG;
    static uint8_t CAN_Send_Data[8];
    uint32_t send_mail_box;

    TX_MSG.StdId = CAN_VTM_DATA_Frame_0;
    TX_MSG.IDE = CAN_ID_STD;
    TX_MSG.RTR = CAN_RTR_DATA;
    TX_MSG.DLC = 0x08;
    CAN_Send_Data[0] = vtm_data[0];
    CAN_Send_Data[1] = vtm_data[1];
    CAN_Send_Data[2] = vtm_data[2];
    CAN_Send_Data[3] = vtm_data[3];
    CAN_Send_Data[4] = vtm_data[4];
    CAN_Send_Data[5] = vtm_data[5];
    CAN_Send_Data[6] = vtm_data[6];
    CAN_Send_Data[7] = vtm_data[7];

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
    HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);

    TX_MSG.StdId = CAN_VTM_DATA_Frame_1;
    CAN_Send_Data[0] = vtm_data[8];
    CAN_Send_Data[1] = vtm_data[9];
    CAN_Send_Data[2] = vtm_data[10];
    CAN_Send_Data[3] = vtm_data[11];
    CAN_Send_Data[4] = 0;
    CAN_Send_Data[5] = 0;
    CAN_Send_Data[6] = 0;
    CAN_Send_Data[7] = 0;

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
    HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
}

void Send_Robot_Info(CAN_HandleTypeDef *_hcan, int8_t ID, uint16_t heatLimit, uint16_t heat, uint16_t bulletSpeed, uint8_t level, uint8_t game_progress)
{
    static CAN_TxHeaderTypeDef TX_MSG;
    static uint8_t CAN_Send_Data[8];
    uint32_t send_mail_box;

    TX_MSG.StdId = 0x133;
    TX_MSG.IDE = CAN_ID_STD;
    TX_MSG.RTR = CAN_RTR_DATA;
    TX_MSG.DLC = 0x08;
    if (game_progress & 0x04)
    {
        CAN_Send_Data[0] = ID | (1 << 7);
    }
    else
    {
        CAN_Send_Data[0] = ID | (0 << 7);
    }

    CAN_Send_Data[1] = (heatLimit >> 8) & 0xFF;
    CAN_Send_Data[2] = heatLimit & 0xFF;
    CAN_Send_Data[3] = (heat >> 8) & 0xFF;
    CAN_Send_Data[4] = heat & 0xFF;
    CAN_Send_Data[5] = (bulletSpeed >> 8) & 0xFF;
    CAN_Send_Data[6] = bulletSpeed & 0xFF;
    CAN_Send_Data[7] = level;

    if (is_TOE_Error(JUDGE_TOE))
        CAN_Send_Data[7] = 0;

    while (!((_hcan->State == HAL_CAN_STATE_READY) || (_hcan->State == HAL_CAN_STATE_LISTENING)))
    {
    }
    while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0)
    {
    }
    /* Check Tx Mailbox 1 status */
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
    HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
}

void Send_Reset_Command(CAN_HandleTypeDef *_hcan)
{
    static CAN_TxHeaderTypeDef TX_MSG;
    static uint8_t CAN_Send_Data[8];
    uint32_t send_mail_box;

    TX_MSG.StdId = CAN_SYSTEM_RESET_CMD;
    TX_MSG.IDE = CAN_ID_STD;
    TX_MSG.RTR = CAN_RTR_DATA;
    TX_MSG.DLC = 0x08;

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
    HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
}

void Send_Power_Data(CAN_HandleTypeDef *_hcan, uint16_t Chassis_power_limit, uint16_t Chassis_power_buffer)
{
    if (Chassis_power_limit >= 10240)
        Chassis_power_limit /= 256;
    if (Chassis_power_limit >= 200)
        Chassis_power_limit /= 5;

    if(Chassis_power_limit > 80)
        Chassis_power_limit = 80;
        
    static CAN_TxHeaderTypeDef TX_MSG;
    static uint8_t CAN_Send_Data[8];
    uint32_t send_mail_box;

    TX_MSG.StdId = CAN_POWER_INFO_ID;
    TX_MSG.IDE = CAN_ID_STD;
    TX_MSG.RTR = CAN_RTR_DATA;
    TX_MSG.DLC = 0x08;
    CAN_Send_Data[0] = Chassis_power_limit >> 8;
    CAN_Send_Data[1] = Chassis_power_limit;
    CAN_Send_Data[2] = Chassis_power_buffer >> 8;
    CAN_Send_Data[3] = Chassis_power_buffer;
    CAN_Send_Data[4] = 0;
    CAN_Send_Data[5] = 0;
    CAN_Send_Data[6] = 0;
    CAN_Send_Data[7] = 0;

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
    HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
}

void Send_Chassis_IMU(CAN_HandleTypeDef *_hcan, float Gyro_z)
{
    static CAN_TxHeaderTypeDef TX_MSG;
    static uint8_t CAN_Send_Data[8] = {0};
    uint32_t send_mail_box;
    TX_MSG.StdId = CAN_Chassis_IMU;
    TX_MSG.IDE = CAN_ID_STD;
    TX_MSG.RTR = CAN_RTR_DATA;
    TX_MSG.DLC = 0x08;

    memcpy(CAN_Send_Data, &Gyro_z, sizeof(float));

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
    HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
}
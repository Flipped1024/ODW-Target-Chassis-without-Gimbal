#include "chassis_task.h"

Chassis_t Chassis = {0};

float dt = 0;
float t = 0;

void Chassis_Control(void)
{
    dt = DWT_GetDeltaT(&Chassis.Chassis_DWT_Count);
    t += dt;
    // Get the angle of Chassis and Gimbal
    Chassis_Get_Theta();
    // Set Chassis's mode
    Chassis_Set_Mode();
    // Handle the data from keyboard and remote control
    Chassis_Get_CtrlValue();
    // Chassis calcutation
    Chassis_Set_Control();
    // Send Chassis current to Motor
    Send_Chassis_Current();
}

void Chassis_Init(void)
{
    /*Chassis struct init*/
    Chassis.WheelRadius = 0.0765;
    Chassis.WheelReductionRatio = 17.0 / 268.0;
    Chassis.VX_k = 2000;
    Chassis.VY_k = 2000;
    Chassis.VR_k = 500;
    Chassis.VelocityRatio = VELOCITY_RATIO;
    Chassis.rcStickRotateRatio = 0.5f;
    Chassis.rcMouseRotateRatio = 0.05f;
    Chassis.YawCorrectionScale = 0;
    Chassis.HeadingFlag = 0;
    Chassis.Heading = 0;
    Chassis.GravityCenter_Adjustment = 1;
    Chassis.Is_Attitude_Control_On = FALSE;
    Chassis.Mode = Silence_Mode;
    for (uint8_t i = 0; i < 4; i++)
        Chassis.Attitude_adjustment[i] = 1.0f;

    /* PID Init */
    for (uint8_t i = 0; i < 4; i++)
    {
        PID_Init(&Chassis.ChassisMotor[i].PID_Velocity, 16384, 16384, 0, 15, 60, 0, 500, 100, 0.001, 0, 1, Integral_Limit | OutputFilter);
        Chassis.ChassisMotor[i].Max_Out = 16384;
    }
    PID_Init(&Chassis.Vx_Compensate, 100, 0, 0, 1.0, 0.0, 0, 0, 0, 0, 0, 0, 0);
    PID_Init(&Chassis.Vy_Compensate, 100, 0, 0, 1.0, 0.0, 0, 0, 0, 0, 0, 0, 0);
    PID_Init(&Chassis.Vr_Compensate, 50, 0, 0, 2.0, 0.0, 0.0, 0, 0, 0.3, 0.3, 0, OutputFilter | DerivativeFilter);

    PID_Init(&Chassis.RotateFollow, 350, 0, 0, 10.0f, 0.0f, 0.5f, 0, 0, 0, 0, 0, Integral_Limit | OutputFilter);

    /*Power control init*/
    Chassis.Spinning_direction = 1;
    Power_Control.Is_Cap_On = FALSE;
    Power_Control.Is_Cap_Used = TRUE;

    initChassisFusion(&chassis_fusion);

    /* 重置参数初始化 */
    Chassis.yaw_offset_ = 0.0f; 
    Chassis.TotalTheta = 0;
    Chassis.Theta = 0;

    //Auto
    Chassis.displacement_x_ = 0.0f;
    Chassis.displacement_y_ = 0.0f;
    Chassis.traverse_target_distance_ = 1.5f; 
    Chassis.traverse_direction_ = 1;          
}

void Chassis_Get_Theta(void)
{
    float temp_yaw_ = AHRS.Yaw - Chassis.yaw_offset_;
    
    // 约束到 -180 ~ +180
    while (temp_yaw_ > 180.0f) temp_yaw_ -= 360.0f;
    while (temp_yaw_ < -180.0f) temp_yaw_ += 360.0f;

    Chassis.TotalTheta = temp_yaw_;
    Chassis.Theta = Chassis.TotalTheta;

    Chassis.FollowTheta = Chassis.Theta;
}

void Chassis_Set_Mode(void)
{
    static uint16_t LastKeyCode = 0;
    static uint16_t LastRightSwitch = 0;

    static uint8_t last_left_switch_ = 0;
    if (remote_control.switch_left == Switch_Down && last_left_switch_ != Switch_Down)
    {
        Chassis.yaw_offset_ = AHRS.Yaw; 

        Chassis.Target_Yaw = 0.0f;      
        
        Chassis.displacement_x_ = 0.0f;
        Chassis.displacement_y_ = 0.0f;
    }
    last_left_switch_ = remote_control.switch_left;

    /*Set mode by key_code*/
    if ((remote_control.key_code & Key_E) && (remote_control.key_code & Key_F))
        Chassis.Mode = Silence_Mode;

    if ((remote_control.key_code & Key_F) && !(LastKeyCode & Key_F))
    {
        if (Chassis.Mode != Spinning_Mode)
            Chassis.Mode = Spinning_Mode;
        else
            Chassis.Mode = Follow_Mode;

        Chassis.Spinning_direction *= -1;
    }

    if ((remote_control.key_code & Key_V) && !(LastKeyCode & Key_V))
        Chassis.sentry = 1;
    if(remote_control.key_code & Key_W)
        Chassis.sentry = 0;

    if(Chassis.sentry == 1)
        Chassis.Mode = Spinning_Mode;

    // If motor lost, Silence_Mode
    if ((is_TOE_Error(CHASSIS_MOTOR1_TOE) && is_TOE_Error(CHASSIS_MOTOR2_TOE) && is_TOE_Error(CHASSIS_MOTOR3_TOE) && is_TOE_Error(CHASSIS_MOTOR4_TOE)))
        Chassis.Mode = Silence_Mode;

    /*Set mode by switch */
    /*Set mode by switch */
    if (remote_control.switch_right == Switch_Up)
    {
        Chassis.Mode = Follow_Mode;
    }
    else if (remote_control.switch_right == Switch_Middle)
    {
        Chassis.Mode = Silence_Mode; 
    }
    else if (remote_control.switch_right == Switch_Down)
    {
        if (LastRightSwitch != Switch_Down)
        {
            Chassis.Spinning_direction *= -1;
        }

        if (remote_control.switch_left == Switch_Up)
        {
            Chassis.Mode = Var_Spinning_Mode; 
        }
        else
        {
            Chassis.Mode = Spinning_Mode;
        }
    }

    /*Cap Switch*/
    if ((remote_control.key_code & Key_SHIFT || remote_control.key_code & Key_CTRL) && (Cap.Voltage > CAP_MIN_VOLTAGE))
        Power_Control.Is_Cap_On = TRUE;
    else
        Power_Control.Is_Cap_On = FALSE;

    /*Refresh*/
    LastKeyCode = remote_control.key_code;
    LastRightSwitch = remote_control.switch_right;
}

void Chassis_Get_CtrlValue(void)
{
    static float Press_Timestamp_W, Press_Timestamp_S, Press_Timestamp_A, Press_Timestamp_D;
    static uint16_t LastKeyCode = 0;

    static float Temp_Vx, Temp_Vy; // Velocity_Ref
    static float tempVal;
    static float RC = 0.000001f;

    static float cap_ratio = 1.0f;
    const float cap_fuck = 0.001f;
    
    // 如果裁判系统丢失，限制功率
    if (is_TOE_Error(JUDGE_TOE))
    {
        robot_state.chassis_power_limit = 45.0f;
        robot_state.shooter_barrel_cooling_value = 10.0f;
        robot_state.shooter_barrel_heat_limit = 50.0f;
        power_heat_data.buffer_energy = 30.0f;
    }

    if (robot_state.chassis_power_limit >= 10240) 
        robot_state.chassis_power_limit /= 256;
    if (robot_state.chassis_power_limit > 120) 
        robot_state.chassis_power_limit = 120;
    if (robot_state.chassis_power_limit == 0)
        robot_state.chassis_power_limit = 45;

    // if (Chassis.Mode == Auto_Traverse_Mode)
    // {
        // float auto_speed_ = 400.0f; // 相当于摇杆满额度的设定速度，可微调
        
        // if (Chassis.traverse_direction_ == 1)
        // {
        //     Temp_Vx = auto_speed_;
        //     if (Chassis.displacement_x_ > Chassis.traverse_target_distance_)
        //     {
        //         Chassis.traverse_direction_ = -1; // 触达边界，反弹
        //     }
        // }
        // else
        // {
        //     Temp_Vx = -auto_speed_;
        //     if (Chassis.displacement_x_ < -Chassis.traverse_target_distance_)
        //     {
        //         Chassis.traverse_direction_ = 1;
        //     }
        // }
        // Temp_Vy = 0.0f; // 横向移动，前后不给速度
        // LastKeyCode = remote_control.key_code; 
        
    // }
    // else
    // {
        /*Key_code WSAD*/
        if (remote_control.key_code & Key_A || remote_control.key_code & Key_D)
        {
            if (!(LastKeyCode & Key_A) && (remote_control.key_code & Key_A))
                Press_Timestamp_A = USER_GetTick();
            if (remote_control.key_code & Key_A)
            {
                if (USER_GetTick() - Press_Timestamp_A > 100)
                    Temp_Vx = -660.0f;
                else
                    Temp_Vx = -220.0f;
            }

            if (!(LastKeyCode & Key_D) && (remote_control.key_code & Key_D))
                Press_Timestamp_D = USER_GetTick();
            if (remote_control.key_code & Key_D)
            {
                if (USER_GetTick() - Press_Timestamp_D > 100)
                    Temp_Vx = 660.0f;
                else
                    Temp_Vx = 220.0f;
            }
        }
        else
            Temp_Vx = 0;

        if (remote_control.key_code & Key_W || remote_control.key_code & Key_S)
        {
            if (!(LastKeyCode & Key_W) && (remote_control.key_code & Key_W))
                Press_Timestamp_W = USER_GetTick();
            if (remote_control.key_code & Key_W)
            {
                if (USER_GetTick() - Press_Timestamp_W > 100)
                    Temp_Vy = 660.0f;
                else
                    Temp_Vy = 220.0f;
            }

            if (!(LastKeyCode & Key_S) && (remote_control.key_code & Key_S))
                Press_Timestamp_S = USER_GetTick();
            if (remote_control.key_code & Key_S)
            {
                if (USER_GetTick() - Press_Timestamp_S > 100)
                    Temp_Vy = -660.0f;
                else
                    Temp_Vy = -220.0f;
            }
        }
        else
            Temp_Vy = 0;

        LastKeyCode = remote_control.key_code; 
        
        Temp_Vx += remote_control.ch3;
        Temp_Vy += remote_control.ch4;
    // }

    if (Power_Control.Is_Cap_On == TRUE)
    {
        if (cap_ratio < 1.0f)
            cap_ratio += cap_fuck;
        else
            cap_ratio = 1.0f;
        Temp_Vx *= (cap_ratio * 1.5f);
        Temp_Vy *= (cap_ratio * 1.5f);
    }
    else
    {
        cap_ratio = 0.0f;
        Temp_Vy = 1.5 * Temp_Vy;
    }
    if (fabsf(Temp_Vx) > 1e-3f)
    {
        if (Chassis.Vx * Temp_Vx < 0)
        {
            Chassis.Vx = 0;
        }
        tempVal = (Temp_Vx - Chassis.Vx) / (RC + dt);
        if (tempVal > Chassis.VX_k)
            tempVal = Chassis.VX_k;
        else if (tempVal < -Chassis.VX_k)
            tempVal = -Chassis.VX_k;
        Chassis.Vx += tempVal * dt;
    }
    else
    {
        tempVal = (Temp_Vx - Chassis.Vx) / (0.01f + dt);
        Chassis.Vx += tempVal * dt;
    }

    if (fabsf(Temp_Vy) > 1e-3f)
    {
        tempVal = (Temp_Vy - Chassis.Vy) / (RC + dt);
        if (tempVal > Chassis.VY_k)
            tempVal = Chassis.VY_k;
        else if (tempVal < -Chassis.VY_k)
            tempVal = -Chassis.VY_k;
        Chassis.Vy += tempVal * dt;
    }
    else
    {
        tempVal = (Temp_Vy - Chassis.Vy) / (0.01f + dt);
        Chassis.Vy += tempVal * dt;
    }
}

void Chassis_Set_Control(void)
{
    const float lpf_a = 2 * PI * 4 * dt / (1 + 2 * PI * 4 * dt);
    const float lpf_r = 2 * PI * 1000 * dt / (1 + 2 * PI * 1000 * dt);

    static float vx = 0, vy = 0, vr = 0;
    static float ovx = 0, ovy = 0; 

    Chassis.Heading = 0.0f;

    static uint8_t last_mode = Silence_Mode;
    if (Chassis.Mode == Follow_Mode && last_mode != Follow_Mode)
    {
        Chassis.Target_Yaw = Chassis.TotalTheta; 
    }
    last_mode = Chassis.Mode;

    // 坐标系反向投影前角
    Chassis.DeflectionAngle = (Chassis.FollowTheta - Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE) / RADIAN_COEF;

    float world_vx_rpm_ = user_cos(-Chassis.DeflectionAngle) * chassis_fusion.filtered_vx_ - user_sin(-Chassis.DeflectionAngle) * chassis_fusion.filtered_vy_;
    float world_vy_rpm_ = user_sin(-Chassis.DeflectionAngle) * chassis_fusion.filtered_vx_ + user_cos(-Chassis.DeflectionAngle) * chassis_fusion.filtered_vy_;
    
    // ( V = RPM * 2πR / 60 )
    float rpm_to_ms_coef_ = (2.0f * PI * Chassis.WheelRadius) / 60.0f;
    
    Chassis.displacement_x_ += world_vx_rpm_ * rpm_to_ms_coef_ * dt;
    Chassis.displacement_y_ += world_vy_rpm_ * rpm_to_ms_coef_ * dt;

    switch (Chassis.Mode)
    {
    case Follow_Mode:
    {
        float rc_vr = -remote_control.ch1 * Chassis.rcStickRotateRatio;
        
        if (fabsf(rc_vr) > 5.0f) 
        {
            Chassis.Vr = rc_vr;
            Chassis.Target_Yaw = Chassis.TotalTheta; 
        }
        else 
        {
            float error = Chassis.Target_Yaw - Chassis.TotalTheta;
            
            while (error > 180.0f)  error -= 360.0f;
            while (error < -180.0f) error += 360.0f;
            
            Chassis.Vr = PID_Calculate(&Chassis.RotateFollow, 0.0f, error);
        }
        
        // 限幅保护
        if (Chassis.Vr > 350) Chassis.Vr = 350;
        if (Chassis.Vr < -350) Chassis.Vr = -350;

        Chassis.VxTransfer = Chassis.Vx;
        Chassis.VyTransfer = Chassis.Vy;
        Chassis.GravityCenter_Adjustment = GRAVITYCENTER_ADJUSTMENT;
        break;
    }
    case Silence_Mode:
        Chassis.Vr = 0;
        Chassis.VxTransfer = 0;
        Chassis.VyTransfer = 0;

        // Press W|S|A|D in Silence_Mode, set Follow_Mode
        if ((remote_control.key_code & Key_W) || (remote_control.key_code & Key_A) || (remote_control.key_code & Key_S) || (remote_control.key_code & Key_D))
            Chassis.Mode = Follow_Mode;
        break;

        case Var_Spinning_Mode:
        {
            static uint32_t spin_update_count = 0;
            static float target_rand_amp = 112.0f; 
            static float smooth_rand_amp = 112.0f; 
    
            if (spin_update_count++ % 250 == 0)
            {
                target_rand_amp = rand() % 25 + 100;
            }
    
            smooth_rand_amp += (target_rand_amp - smooth_rand_amp) * 0.05f;
    
            if (Power_Control.Is_Cap_On == TRUE)
                Chassis.Vr = (int16_t)(CAP_SPINNING_B + smooth_rand_amp * user_sin(CAP_SPINNING_OMEGA * t));
            else
                Chassis.Vr = (int16_t)(SPINNING_B + smooth_rand_amp * user_sin(SPINNING_OMEGA * t));

            if ((remote_control.key_code & Key_W) || (remote_control.key_code & Key_A) || 
                (remote_control.key_code & Key_S) || (remote_control.key_code & Key_D) ||
                (remote_control.ch1 != 0) || (remote_control.ch2 != 0) || 
                (remote_control.ch3 != 0) || (remote_control.ch4 != 0))
            {
                Chassis.Vr = SPINNING_SPEED;
            }
            Chassis.Vr *= Chassis.Spinning_direction;
    
            float target_vx_trans = user_cos(Chassis.DeflectionAngle) * Chassis.Vx + 
                                    user_sin(Chassis.DeflectionAngle) * Chassis.Vy;
                                    
            float target_vy_trans = -user_sin(Chassis.DeflectionAngle) * Chassis.Vx + 
                                        user_cos(Chassis.DeflectionAngle) * Chassis.Vy;
    
            Chassis.VxTransfer = target_vx_trans + PID_Calculate(&Chassis.Vx_Compensate, chassis_fusion.filtered_vx_, target_vx_trans);
            Chassis.VyTransfer = target_vy_trans + PID_Calculate(&Chassis.Vy_Compensate, chassis_fusion.filtered_vy_, target_vy_trans);
    
            Chassis.VxTransfer *= 0.5f;
            Chassis.VyTransfer *= 0.5f;
    
            Chassis.GravityCenter_Adjustment = 1.0f;
            break;
        }
    
        case Spinning_Mode:
        {  
            Chassis.Vr = SPINNING_B;
    
            if ((remote_control.key_code & Key_W) || (remote_control.key_code & Key_A) || 
                (remote_control.key_code & Key_S) || (remote_control.key_code & Key_D) ||
                (remote_control.ch1 != 0) || (remote_control.ch2 != 0) || 
                (remote_control.ch3 != 0) || (remote_control.ch4 != 0))
            {
                Chassis.Vr = SPINNING_SPEED;
            }
            Chassis.Vr *= Chassis.Spinning_direction;
    
            float target_vx_trans = user_cos(Chassis.DeflectionAngle) * Chassis.Vx + 
                                    user_sin(Chassis.DeflectionAngle) * Chassis.Vy;
                                    
            float target_vy_trans = -user_sin(Chassis.DeflectionAngle) * Chassis.Vx + 
                                        user_cos(Chassis.DeflectionAngle) * Chassis.Vy;
    
            Chassis.VxTransfer = target_vx_trans + PID_Calculate(&Chassis.Vx_Compensate, chassis_fusion.filtered_vx_, target_vx_trans);
            Chassis.VyTransfer = target_vy_trans + PID_Calculate(&Chassis.Vy_Compensate, chassis_fusion.filtered_vy_, target_vy_trans);
    
            Chassis.VxTransfer *= 0.5f;
            Chassis.VyTransfer *= 0.5f;
    
            Chassis.GravityCenter_Adjustment = 1.0f;
            break;
        }
    }

    if (Chassis.Is_Attitude_Control_On == TRUE) 
    {
        Chassis.Attitude_adjustment[0] = 1.2;
        Chassis.Attitude_adjustment[1] = 1.2;
        Chassis.Attitude_adjustment[2] = 1;
        Chassis.Attitude_adjustment[3] = 1;
    }
    else
    {
        Chassis.Attitude_adjustment[0] = 1;
        Chassis.Attitude_adjustment[1] = 1;
        Chassis.Attitude_adjustment[2] = 1;
        Chassis.Attitude_adjustment[3] = 1;
    }

    if (!(Chassis.Mode == Spinning_Mode || Chassis.Mode == Var_Spinning_Mode))
    {
        Chassis.VxTransfer = lpf_a * Chassis.VxTransfer + (1 - lpf_a) * vx;
        Chassis.VyTransfer = lpf_a * Chassis.VyTransfer + (1 - lpf_a) * vy;
    }


    /*Inverse Kinematics*/
    Chassis.Observed_Vx = (Chassis.ChassisMotor[FR].Velocity_RPM + Chassis.ChassisMotor[FL].Velocity_RPM - Chassis.ChassisMotor[HL].Velocity_RPM - Chassis.ChassisMotor[HR].Velocity_RPM) * Chassis.WheelReductionRatio * 0.5;
    Chassis.Observed_Vy = -(Chassis.ChassisMotor[FR].Velocity_RPM - Chassis.ChassisMotor[FL].Velocity_RPM - Chassis.ChassisMotor[HL].Velocity_RPM + Chassis.ChassisMotor[HR].Velocity_RPM) * Chassis.WheelReductionRatio * 0.5;

    updateChassisFusion(&chassis_fusion, Chassis.Observed_Vx, Chassis.Observed_Vy, 
        AHRS.Accel[0], AHRS.Accel[1], AHRS.Gyro[2], dt);

    if (Chassis.Mode != Target_Mode)
    {
        Chassis.Vr = lpf_r * Chassis.Vr + (1 - lpf_r) * vr;
    }
    float assign_ratio = 0;


    if (Chassis.Mode == Spinning_Mode || Chassis.Mode == Var_Spinning_Mode || Chassis.Mode == Target_Mode)
    {
        assign_ratio = 12;
        Chassis.VelocityRatio = VELOCITY_RATIO * 0.8f;
    }
    else
    {
        assign_ratio = 8;
        if (fabsf(Chassis.Vx) > 10 && fabsf(Chassis.Vy) > 10)
        {
            Chassis.VelocityRatio = VELOCITY_RATIO * 0.866;
        }
        else
        {
            Chassis.VelocityRatio = VELOCITY_RATIO;
        }
        if (Power_Control.Is_Cap_On == TRUE)
        {
            Chassis.VelocityRatio = VELOCITY_RATIO * 2;
        }
    }
    
    /*Calculation - Coordinate Rotated by -90 degrees*/
    Chassis.V1 = -( Chassis.VxTransfer - Chassis.VyTransfer) * Chassis.VelocityRatio * Chassis.Attitude_adjustment[0] + assign_ratio * Chassis.Vr;
    Chassis.V2 = -( Chassis.VxTransfer + Chassis.VyTransfer) * Chassis.VelocityRatio * Chassis.Attitude_adjustment[1] + assign_ratio * Chassis.Vr;
    Chassis.V3 = -(-Chassis.VxTransfer + Chassis.VyTransfer) * Chassis.VelocityRatio * Chassis.Attitude_adjustment[2] + assign_ratio * Chassis.Vr;
    Chassis.V4 = -(-Chassis.VxTransfer - Chassis.VyTransfer) * Chassis.VelocityRatio * Chassis.Attitude_adjustment[3] + assign_ratio * Chassis.Vr;

    // Adjust Gimbal's Gracity_Center
    Chassis.V3 *= Chassis.GravityCenter_Adjustment;
    Chassis.V4 *= Chassis.GravityCenter_Adjustment;
    
    Velocity_MAXLimit();

    // Motor speed calculation
    Motor_Speed_Calculate(&Chassis.ChassisMotor[0], Chassis.ChassisMotor[0].Velocity_RPM, Chassis.V1);
    Motor_Speed_Calculate(&Chassis.ChassisMotor[1], Chassis.ChassisMotor[1].Velocity_RPM, Chassis.V2);
    Motor_Speed_Calculate(&Chassis.ChassisMotor[2], Chassis.ChassisMotor[2].Velocity_RPM, Chassis.V3);
    Motor_Speed_Calculate(&Chassis.ChassisMotor[3], Chassis.ChassisMotor[3].Velocity_RPM, Chassis.V4);

    /*Power Control*/
    Chassis_Power_Cal();
    if (Power_Control.Is_Cap_Used == TRUE)
    {
        if (Chassis.Fly_Mode == TRUE)
            Chassis_Power_Control_Fly();
        else
            Chassis_Power_Control();
    }
    else if (Power_Control.Is_Cap_Used == FALSE)
        Chassis_Power_Control_Without_Cap();

    vx = Chassis.VxTransfer;
    vy = Chassis.VyTransfer;
    vr = Chassis.Vr;
    ovx = Chassis.Observed_Vx;
    ovy = Chassis.Observed_Vy;
    Cap_unused_correct();
}

void Send_Chassis_Current(void)
{
    static uint8_t count = 0;
    if (count % 10 == 0)
    {
        Send_Power_Data(&hcan1, robot_state.chassis_power_limit, power_heat_data.buffer_energy);
        count = 0;
    }
    count++;

    if (is_TOE_Error(RC_TOE))
    {
        if (Send_Motor_Current_1_4(&hcan1, 0, 0, 0, 0) == HAL_OK)
            HAL_IWDG_Refresh(&hiwdg);
    }
    else
    {
        if (Send_Motor_Current_1_4(&hcan1, Chassis.ChassisMotor[0].Output, Chassis.ChassisMotor[1].Output, Chassis.ChassisMotor[2].Output, Chassis.ChassisMotor[3].Output) == HAL_OK)
            HAL_IWDG_Refresh(&hiwdg);
    }
}

float Max_4(float num1, float num2, float num3, float num4)
{
    float max_num;

    max_num = fabs(num1);
    if (fabs(num2) > max_num)
        max_num = fabs(num2);
    if (fabs(num3) > max_num)
        max_num = fabs(num3);
    if (fabs(num4) > max_num)
        max_num = fabs(num4);

    return max_num;
}

void Velocity_MAXLimit(void)
{
    static float temp_max;
    if (Max_4(fabsf(Chassis.V1), fabsf(Chassis.V2), fabsf(Chassis.V3), fabsf(Chassis.V4)) > MAX_RPM)
    {
        temp_max = Max_4(fabsf(Chassis.V1), fabsf(Chassis.V2), fabsf(Chassis.V3), fabsf(Chassis.V4));

        Chassis.V1 *= MAX_RPM / temp_max;
        Chassis.V2 *= MAX_RPM / temp_max;
        Chassis.V3 *= MAX_RPM / temp_max;
        Chassis.V4 *= MAX_RPM / temp_max;
    }
}
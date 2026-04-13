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

    /*Findheading*/
    // for (uint8_t i = 0; i < 4; i++)
    // {
    //     // Chassis.TotalTheta = (Gimbal.YawMotor.Total_angle - Gimbal.YawMotor.zero_offset) * ENCODERCOEF * YAW_REDUCTION_RATIO + i * YAW_REDUCTION_CORRECTION_ANGLE;
    //     // Chassis.Theta = loop_float_constrain((Chassis.TotalTheta + Gimbal.YawMotor.Angle_offset * ENCODERCOEF * YAW_REDUCTION_RATIO), -180, 180);
    // }

    Chassis.TotalTheta = 0;
    Chassis.Theta = 0;

    /*PID Init*/
    // PID_Velocity
    for (uint8_t i = 0; i < 4; i++)
    {
        PID_Init(&Chassis.ChassisMotor[i].PID_Velocity, 16384, 16384, 0, 15, 60, 0, 500, 100, 0.001, 0, 1, Integral_Limit | OutputFilter);
        Chassis.ChassisMotor[i].Max_Out = 16384;
    }
    PID_Init(&Chassis.Vx_Compensate, 100, 0, 0, 1.0, 0.0, 0, 0, 0, 0, 0, 0, 0);
    PID_Init(&Chassis.Vy_Compensate, 100, 0, 0, 1.0, 0.0, 0, 0, 0, 0, 0, 0, 0);
    PID_Init(&Chassis.Vr_Compensate, 50, 0, 0, 2.0, 0.0, 0.0, 0, 0, 0.3, 0.3, 0, OutputFilter | DerivativeFilter);

    PID_Init(&Chassis.RotateFollow, 350, 0, 0, 10.0f, 0.0f, 0.5f, 0, 0, 0, 0, 0, Integral_Limit | OutputFilter);    //航向PID

    /*Power control init*/
    Chassis.Spinning_direction = 1;
    Power_Control.Is_Cap_On = FALSE;
    Power_Control.Is_Cap_Used = TRUE;
}

void Chassis_Get_Theta(void)
{
    // /*Get the angle of Chassis and Gimbal*/
    // Chassis.TotalTheta = (Gimbal.YawMotor.Total_angle - Gimbal.YawMotor.zero_offset) * ENCODERCOEF + Chassis.YawCorrectionScale * YAW_REDUCTION_CORRECTION_ANGLE;
    // Chassis.Theta = loop_float_constrain((Chassis.TotalTheta + Gimbal.YawMotor.Angle_offset * ENCODERCOEF), -180, 180);

    // /*Finding head*/
    // if (Chassis.Fly_Mode == TRUE)
    // {
    //     Chassis.HeadingFlag = 1; // 在飞坡模式下 选定最优利于飞坡的一面
    //     Chassis.Theta = loop_float_constrain((Chassis.Theta + Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE), -180, 180);
    // }
    // else
    //     Find_heading_2heads_L();

    // // When Gimbal is abnormal,reset the angle
    // if (is_TOE_Error(GIMBAL_YAW_MOTOR_TOE))
    // {
    //     Chassis.TotalTheta = 0;
    //     Chassis.Theta = 0;
    // }

    // Chassis.FollowTheta = float_deadband(Chassis.Theta, -0.005f, 0.005f);


    Chassis.TotalTheta = AHRS.Yaw;
    Chassis.Theta = Chassis.TotalTheta;

    Chassis.FollowTheta = Chassis.Theta;
}

void Chassis_Set_Mode(void)
{
    static uint16_t LastKeyCode = 0;
    static uint16_t LastRightSwitch = 0;
    static uint8_t num = 1;

    /*Set mode by key_code*/
    // Keep EF, Silence_Mode
    if ((remote_control.key_code & Key_E) && (remote_control.key_code & Key_F))
        Chassis.Mode = Silence_Mode;

    // F, switch Spinning_Mode and Follow_Mode
    if ((remote_control.key_code & Key_F) && !(LastKeyCode & Key_F))
    {
        if (Chassis.Mode != Spinning_Mode)
            Chassis.Mode = Spinning_Mode;
        else
            Chassis.Mode = Follow_Mode;

        Chassis.Spinning_direction *= -1;
    }

    if ((remote_control.key_code & Key_V) && !(LastKeyCode & Key_V))
    {
        Chassis.sentry = 1;
    }
    if(remote_control.key_code & Key_W)
    {
        Chassis.sentry = 0;
    }

    if(Chassis.sentry == 1)
    {
        Chassis.Mode = Spinning_Mode;
    }

    // If motor lost, Silence_Mode
    if ((is_TOE_Error(CHASSIS_MOTOR1_TOE) && is_TOE_Error(CHASSIS_MOTOR2_TOE) && is_TOE_Error(CHASSIS_MOTOR3_TOE) && is_TOE_Error(CHASSIS_MOTOR4_TOE)))
        Chassis.Mode = Silence_Mode;

    /*Set mode by switch*/
    if (remote_control.switch_right == Switch_Up)
        Chassis.Mode = Follow_Mode;

    if (remote_control.switch_right == Switch_Down && LastRightSwitch != Switch_Down)
    {
        num++;
        if (num % 3 == 1)
            Chassis.Mode = Silence_Mode;
        else if (num % 3 == 0)
        {
            Chassis.Mode = Spinning_Mode;
            Chassis.Spinning_direction *= -1;
        }
        else
        {
            Chassis.Mode = Target_Mode;
        }
    }

    /*Cap Switch*/
    if ((remote_control.switch_left == Switch_Up || remote_control.key_code & Key_SHIFT || remote_control.key_code & Key_CTRL) && (Cap.Voltage > CAP_MIN_VOLTAGE))
    {
        Power_Control.Is_Cap_On = TRUE;
        // Chassis.Fly_Mode = TRUE;
    }
    else
    {
        Power_Control.Is_Cap_On = FALSE;
        // Chassis.Fly_Mode = FALSE;
    }

    /*Fly_Control*/
    // if (remote_control.key_code & Key_CTRL)
    // {
    //     Chassis.Is_Attitude_Control_On = FALSE;
    //     Chassis.Fly_Mode = TRUE;
    // }
    // else
    // {
    //     Chassis.Is_Attitude_Control_On = FALSE;
    //     Chassis.Fly_Mode = FALSE;
    // }

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
    // If Judge_lost
    if (is_TOE_Error(JUDGE_TOE))
    {
        robot_state.chassis_power_limit = 45.0f;
        robot_state.shooter_barrel_cooling_value = 10.0f;
        robot_state.shooter_barrel_heat_limit = 50.0f;

        power_heat_data.buffer_energy = 30.0f;
    }

    // When the power_limit is different from real data
    if (robot_state.chassis_power_limit >= 10240) // max of data_packet
        robot_state.chassis_power_limit /= 256;
    // Power max limit
    if (robot_state.chassis_power_limit > 120) // min of data_packet
        robot_state.chassis_power_limit = 120;
    // Power min limit
    if (robot_state.chassis_power_limit == 0)
        robot_state.chassis_power_limit = 45;

    // Chassis.VelocityRatio = float_constrain(robot_state.chassis_power_limit / 10.0f + 2, 5, 5); // adjust the ratio

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

    LastKeyCode = remote_control.key_code; // Refresh LastKeyCode

    // Get remote_control's data
    Temp_Vx += remote_control.ch3;
    Temp_Vy += remote_control.ch4;
    // Cap_mode
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
    // static int flag = 0;
    // static int last = 0;
    // // static int compensate = 0;
    const float lpf_a = 2 * PI * 4 * dt / (1 + 2 * PI * 4 * dt);
    const float lpf_r = 2 * PI * 1000 * dt / (1 + 2 * PI * 1000 * dt);
    // static float vx = 0, vy = 0, vr = 0;
    // static float ovx = 0, ovy = 0, ovr = 0;

    static float vx = 0, vy = 0, vr = 0;
    static float ovx = 0, ovy = 0; 

    static int8_t wait = 0;
    Chassis.Heading = 0.0f;

    // 检测模式切换，防止刚切入 Follow_Mode 时底盘疯转
    static uint8_t last_mode = Silence_Mode;
    if (Chassis.Mode == Follow_Mode && last_mode != Follow_Mode)
    {
        // 刚切入 Follow_Mode 时，强制同步目标角为当前真实姿态
        Chassis.Target_Yaw = Chassis.TotalTheta; 
    }
    last_mode = Chassis.Mode;

    switch (Chassis.Mode)
    {
    // case Follow_Mode:
    // FOLLOW:
    //     if (fabsf(Chassis.FollowTheta - Chassis.Heading) > 10.0f)
    //     {
    //         PID_Init(&Chassis.RotateFollow, 300, 0, 0, 20, 0, 0.0, 0, 0, 0, 0, 5, Integral_Limit | Derivative_On_Measurement | OutputFilter | DerivativeFilter);
    //     }
    //     else if (fabsf(Chassis.FollowTheta - Chassis.Heading) > 2.0f)
    //     {
    //         PID_Init(&Chassis.RotateFollow, 300, 0, 0, 3, 0, 0.0, 0, 0, 0, 0, 5, Integral_Limit | Derivative_On_Measurement | OutputFilter | DerivativeFilter);
    //     }
    //     // else
    //     // {
    //     //     PID_Init(&Chassis.RotateFollow, 10, 10, 0, 0.0, 0.0, 0.00, 0, 0, 0.1, 0.1, 5, Integral_Limit | OutputFilter | DerivativeFilter);
    //     // }
    //     Chassis.GravityCenter_Adjustment = GRAVITYCENTER_ADJUSTMENT;

    //     if (Power_Control.Is_Cap_On == TRUE) // Cap_Mode
    //     {
    //         Chassis.Vr = PID_Calculate(&Chassis.RotateFollow, Chassis.FollowTheta, Chassis.Heading);
    //     } // No Feedforward
    //     else
    //     {
    //         Chassis.Vr = PID_Calculate(&Chassis.RotateFollow, Chassis.FollowTheta, Chassis.Heading) + remote_control.ch1 * Chassis.rcStickRotateRatio + remote_control.mouse.x * Chassis.rcMouseRotateRatio;
    //     }

    //     // Transform Gimbal's angle to Chassis
    //     if (Chassis.VxTransfer > 50 || Chassis.VyTransfer > 50)
    //     {
    //         float temp[2] = {
    //             user_cos((Chassis.FollowTheta - Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE) / RADIAN_COEF) * Chassis.Vx + user_sin((Chassis.FollowTheta - Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE) / RADIAN_COEF) * Chassis.Vy,
    //             -user_sin((Chassis.FollowTheta - Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE) / RADIAN_COEF) * Chassis.Vx + user_cos((Chassis.FollowTheta - Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE) / RADIAN_COEF) * Chassis.Vy};
    //         Chassis.VxTransfer = user_cos((Chassis.FollowTheta - Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE) / RADIAN_COEF) * Chassis.Vx + user_sin((Chassis.FollowTheta - Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE) / RADIAN_COEF) * Chassis.Vy + PID_Calculate(&Chassis.Vx_Compensate, Chassis.Observed_Vx, temp[0]);
    //         Chassis.VyTransfer = -user_sin((Chassis.FollowTheta - Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE) / RADIAN_COEF) * Chassis.Vx + user_cos((Chassis.FollowTheta - Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE) / RADIAN_COEF) * Chassis.Vy + PID_Calculate(&Chassis.Vy_Compensate, Chassis.Observed_Vy, temp[1]);
    //     }
    //     else
    //     {
    //         Chassis.VxTransfer = user_cos((Chassis.FollowTheta - Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE) / RADIAN_COEF) * Chassis.Vx + user_sin((Chassis.FollowTheta - Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE) / RADIAN_COEF) * Chassis.Vy;
    //         Chassis.VyTransfer = -user_sin((Chassis.FollowTheta - Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE) / RADIAN_COEF) * Chassis.Vx + user_cos((Chassis.FollowTheta - Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE) / RADIAN_COEF) * Chassis.Vy;
    //     }
    //     break;

    // case Follow_Mode:
    //     // 靶车模式：废除所有跟随PID与三角函数投影
    //     // 遥控器的推杆 (Vx, Vy) 直接映射到底盘的物理 (前后, 左右)
    //     Chassis.Vr = remote_control.ch1 * Chassis.rcStickRotateRatio; // 保留手动自转
    //     Chassis.VxTransfer = Chassis.Vx;
    //     Chassis.VyTransfer = Chassis.Vy;

    //     Chassis.GravityCenter_Adjustment = GRAVITYCENTER_ADJUSTMENT;
    //     break;

    case Follow_Mode:
        float rc_vr = -remote_control.ch1 * Chassis.rcStickRotateRatio;
        
        if (fabsf(rc_vr) > 5.0f) 
        {
            // [有输入]：开环响应摇杆输入，并持续刷新目标角
            Chassis.Vr = rc_vr;
            Chassis.Target_Yaw = Chassis.TotalTheta; 
        }
        else 
        {
            // [无输入]：启动闭环 PID 纠偏
            float error = Chassis.Target_Yaw - Chassis.TotalTheta;
            
            // 处理 -180 到 +180 之间的最短路径过零点问题
            while (error > 180.0f)  error -= 360.0f;
            while (error < -180.0f) error += 360.0f;
            
            Chassis.Vr = PID_Calculate(&Chassis.RotateFollow, 0.0f, error);
            
        }
        
        // float current_yaw_deg = AHRS.Yaw;
        
        // /* 2. 锁定初始目标角 (仅在刚切入 Follow_Mode 时执行一次) */
        // static uint8_t last_mode = Silence_Mode;
        // if (Chassis.Mode == Follow_Mode && last_mode != Follow_Mode)
        // {
        //     Chassis.Target_Yaw = current_yaw_deg; 
        // }
        // last_mode = Chassis.Mode;

        // /* 3. 计算跨零点误差 */
        // float error = Chassis.Target_Yaw - current_yaw_deg;
        // while (error > 180.0f)  error -= 360.0f;
        // while (error < -180.0f) error += 360.0f;
        
        // /* 4. 纯比例控制闭环 (如果底盘发生正反馈疯转，将 -error 改为 error) */
        // Chassis.Vr = PID_Calculate(&Chassis.RotateFollow, 0.0f, error);
        
        // 限幅保护
        if (Chassis.Vr > 350) Chassis.Vr = 350;
        if (Chassis.Vr < -350) Chassis.Vr = -350;

        Chassis.VxTransfer = Chassis.Vx;
        Chassis.VyTransfer = Chassis.Vy;
        Chassis.GravityCenter_Adjustment = GRAVITYCENTER_ADJUSTMENT;
        break;

    case Silence_Mode:
        Chassis.Vr = 0;
        Chassis.VxTransfer = 0;
        Chassis.VyTransfer = 0;

        // Press W|S|A|D in Silence_Mode, set Follow_Mode
        if ((remote_control.key_code & Key_W) || (remote_control.key_code & Key_A) || (remote_control.key_code & Key_S) || (remote_control.key_code & Key_D))
            Chassis.Mode = Follow_Mode;
        break;

        case Spinning_Mode:
        {  
            // srand(t);
            // static float rand_amp;
            // rand_amp = rand() % 25 + 100;

            // if (Power_Control.Is_Cap_On == TRUE)
            //     Chassis.Vr = (int16_t)(CAP_SPINNING_B + rand_amp * user_sin(fabsf(CAP_SPINNING_OMEGA * user_cos(rand_amp)) * t));
            // else
            //     Chassis.Vr = (int16_t)(SPINNING_B + rand_amp * user_sin(fabsf(SPINNING_OMEGA * user_cos(rand_amp)) * t));

            static uint32_t spin_update_count = 0;
            static float target_rand_amp = 112.0f; 
            static float smooth_rand_amp = 112.0f; 
    
            // 500ms
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
    
            // 坐标系反向投影 
            Chassis.DeflectionAngle = (Chassis.FollowTheta - Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE) / RADIAN_COEF;
    
            float target_vx_trans = user_cos(Chassis.DeflectionAngle) * Chassis.Vx + 
                                    user_sin(Chassis.DeflectionAngle) * Chassis.Vy;
                                    
            float target_vy_trans = -user_sin(Chassis.DeflectionAngle) * Chassis.Vx + 
                                        user_cos(Chassis.DeflectionAngle) * Chassis.Vy;
    
            Chassis.VxTransfer = target_vx_trans + PID_Calculate(&Chassis.Vx_Compensate, Chassis.Observed_Vx, target_vx_trans);
            Chassis.VyTransfer = target_vy_trans + PID_Calculate(&Chassis.Vy_Compensate, Chassis.Observed_Vy, target_vy_trans);
    
            Chassis.VxTransfer *= 0.5f;
            Chassis.VyTransfer *= 0.5f;
    
            Chassis.GravityCenter_Adjustment = 1.0f;
            break;
        }
    }
    // if (is_TOE_Error(GIMBAL_YAW_MOTOR_TOE)) // If Gimbal Yaw lost, Control Chassis
    //     Chassis.Vr = remote_control.ch1 * Chassis.rcStickRotateRatio + remote_control.mouse.x * Chassis.rcMouseRotateRatio;

    if (Chassis.Is_Attitude_Control_On == TRUE) // Flying slope
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

    if (!(Chassis.Mode == Spinning_Mode))
    {
        Chassis.VxTransfer = lpf_a * Chassis.VxTransfer + (1 - lpf_a) * vx;
        Chassis.VyTransfer = lpf_a * Chassis.VyTransfer + (1 - lpf_a) * vy;
    }

    // Chassis.Observed_Vx = (Chassis.ChassisMotor[FR].Velocity_RPM - Chassis.ChassisMotor[FL].Velocity_RPM - Chassis.ChassisMotor[HL].Velocity_RPM + Chassis.ChassisMotor[HR].Velocity_RPM) * Chassis.WheelReductionRatio * 0.5;
    // Chassis.Observed_Vy = (Chassis.ChassisMotor[FR].Velocity_RPM + Chassis.ChassisMotor[FL].Velocity_RPM - Chassis.ChassisMotor[HL].Velocity_RPM - Chassis.ChassisMotor[HR].Velocity_RPM) * Chassis.WheelReductionRatio * 0.5;

    /*Inverse Kinematics - Aligned with Rotated Coordinates*/
    Chassis.Observed_Vx = (Chassis.ChassisMotor[FR].Velocity_RPM + Chassis.ChassisMotor[FL].Velocity_RPM - Chassis.ChassisMotor[HL].Velocity_RPM - Chassis.ChassisMotor[HR].Velocity_RPM) * Chassis.WheelReductionRatio * 0.5;
    Chassis.Observed_Vy = -(Chassis.ChassisMotor[FR].Velocity_RPM - Chassis.ChassisMotor[FL].Velocity_RPM - Chassis.ChassisMotor[HL].Velocity_RPM + Chassis.ChassisMotor[HR].Velocity_RPM) * Chassis.WheelReductionRatio * 0.5;

    Chassis.Observed_Vx = lpf_a * Chassis.Observed_Vx + (1 - lpf_a) * ovx;
    Chassis.Observed_Vy = lpf_a * Chassis.Observed_Vy + (1 - lpf_a) * ovy;
    if (Chassis.Mode != Target_Mode)
    {
        Chassis.Vr = lpf_r * Chassis.Vr + (1 - lpf_r) * vr;
    }
    float assign_ratio = 0;
    // float average_motor_current = 0;
    // average_motor_current = (fabs(Chassis.ChassisMotor[0].Current) + fabs(Chassis.ChassisMotor[1].Current) + fabs(Chassis.ChassisMotor[2].Current) + fabs(Chassis.ChassisMotor[3].Current)) / 4;

    if (Chassis.Mode == Spinning_Mode || Chassis.Mode == Target_Mode)
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
        // else if (Gimbal_Date.is_slope == 1)
        // {

        //     if (average_motor_current > 5.3)
        //     {
        //         Chassis.VelocityRatio = VELOCITY_RATIO * 3;
        //         assign_ratio = 3;
        //         Chassis.Upslope_state = 0;
        //     }
        //     else if (average_motor_current > 4 && average_motor_current < 5)
        //     {
        //         Chassis.VelocityRatio = VELOCITY_RATIO * 2;
        //         assign_ratio = 3;
        //         Chassis.Upslope_state = 1;
        //     }
        //     else
        //     {
        //         Chassis.VelocityRatio = VELOCITY_RATIO * 0.8;
        //         assign_ratio = 8;
        //         Chassis.Upslope_state = 2;
        //     }
        // }
        else
        {
            Chassis.VelocityRatio = VELOCITY_RATIO;
        }
        if (Power_Control.Is_Cap_On == TRUE)
        {
            Chassis.VelocityRatio = VELOCITY_RATIO * 2;
        }
    }
    /*Calculation*/
    // Chassis.V1 = -(-Chassis.VxTransfer - Chassis.VyTransfer) * Chassis.VelocityRatio * Chassis.Attitude_adjustment[0] + assign_ratio * Chassis.Vr;
    // Chassis.V2 = -(Chassis.VxTransfer - Chassis.VyTransfer) * Chassis.VelocityRatio * Chassis.Attitude_adjustment[1] + assign_ratio * Chassis.Vr;
    // Chassis.V3 = -(Chassis.VxTransfer + Chassis.VyTransfer) * Chassis.VelocityRatio * Chassis.Attitude_adjustment[2] + assign_ratio * Chassis.Vr;
    // Chassis.V4 = -(-Chassis.VxTransfer + Chassis.VyTransfer) * Chassis.VelocityRatio * Chassis.Attitude_adjustment[3] + assign_ratio * Chassis.Vr;

    // /*Calculation - Coordinate Rotated by -90 degrees*/
    Chassis.V1 = -( Chassis.VxTransfer - Chassis.VyTransfer) * Chassis.VelocityRatio * Chassis.Attitude_adjustment[0] + assign_ratio * Chassis.Vr;
    Chassis.V2 = -( Chassis.VxTransfer + Chassis.VyTransfer) * Chassis.VelocityRatio * Chassis.Attitude_adjustment[1] + assign_ratio * Chassis.Vr;
    Chassis.V3 = -(-Chassis.VxTransfer + Chassis.VyTransfer) * Chassis.VelocityRatio * Chassis.Attitude_adjustment[2] + assign_ratio * Chassis.Vr;
    Chassis.V4 = -(-Chassis.VxTransfer - Chassis.VyTransfer) * Chassis.VelocityRatio * Chassis.Attitude_adjustment[3] + assign_ratio * Chassis.Vr;

    // Adjust Gimbal's Gracity_Center
    Chassis.V3 *= Chassis.GravityCenter_Adjustment;
    Chassis.V4 *= Chassis.GravityCenter_Adjustment;
    // last = remote_control.switch_right;
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
    /*Send Judgement's data to Cap*/
    static uint8_t count = 0;
    if (count % 10 == 0)
    {
        Send_Power_Data(&hcan1, robot_state.chassis_power_limit, power_heat_data.buffer_energy);
        count = 0;
    }
    count++;

    /*Send Chassis current to motors*/
    // If RC and VTM lost, stop the robot

    // if (is_TOE_Error(RC_TOE) && is_TOE_Error(VTM_TOE))
    // {
    //     if (Send_Motor_Current_1_4(&hcan1, 0, 0, 0, 0) == HAL_OK)
    //         HAL_IWDG_Refresh(&hiwdg);
    // }
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

// void Find_heading_4heads(void)
// {
//     // Search the direction of Gimbal
//     if (Chassis.Heading < 0.01f)
//     {
//         if ((Chassis.Mode != Silence_Mode) && ((Chassis.Theta > 45 + Chassis.Heading) && (Chassis.Theta < 135 + Chassis.Heading)))
//         {
//             Chassis.HeadingFlag = 1;

//             Chassis.Theta = loop_float_constrain((Chassis.Theta + Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE),
//                                                  -180, 180);
//         }
//         else if (((Chassis.Mode != Silence_Mode) && (fabsf(Chassis.Theta) > 135) && (Chassis.Heading == 0)) || ((Chassis.Mode != Silence_Mode) && (Chassis.Heading != 0) && (Chassis.Theta < -135 + Chassis.Heading)))
//         {

//             Chassis.HeadingFlag = 2;

//             Chassis.Theta = loop_float_constrain((Chassis.Theta + Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE),
//                                                  -180, 180);
//         }
//         else if ((Chassis.Mode != Silence_Mode) && (Chassis.Theta > -135 + Chassis.Heading) && (Chassis.Theta < -45 + Chassis.Heading))
//         {

//             Chassis.HeadingFlag = 1;

//             Chassis.Theta = loop_float_constrain((Chassis.Theta + Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE),
//                                                  -180, 180);
//         }
//         else if ((Chassis.Mode != Silence_Mode) && (Chassis.Theta < 45 + Chassis.Heading) && (Chassis.Theta > -45 + Chassis.Heading))
//         {
//             Chassis.HeadingFlag = 0;

//             Chassis.Theta = loop_float_constrain((Chassis.Theta + Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE),
//                                                  -180, 180);
//         }
//     }
//     else
//     {
//         if ((Chassis.Mode != Silence_Mode) && ((Chassis.Theta > 45 + Chassis.Heading) && (Chassis.Theta < 135 + Chassis.Heading)))
//         {
//             Chassis.HeadingFlag = -1;

//             Chassis.Theta = loop_float_constrain((Chassis.Theta + Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE),
//                                                  -180, 180);
//         }
//         else if ((Chassis.Mode != Silence_Mode) && (Chassis.Theta < -135 + Chassis.Heading))
//         {

//             Chassis.HeadingFlag = 2;

//             Chassis.Theta = loop_float_constrain((Chassis.Theta + Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE),
//                                                  -180, 180);
//         }
//         else if ((Chassis.Mode != Silence_Mode) && (Chassis.Theta > -135 + Chassis.Heading) && (Chassis.Theta < -45 + Chassis.Heading))
//         {

//             Chassis.HeadingFlag = 1;

//             Chassis.Theta = loop_float_constrain((Chassis.Theta + Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE),
//                                                  -180, 180);
//         }
//         else if ((Chassis.Mode != Silence_Mode) && (Chassis.Theta < 45 + Chassis.Heading) && (Chassis.Theta > -45 + Chassis.Heading))
//         {
//             Chassis.HeadingFlag = 0;

//             Chassis.Theta = loop_float_constrain((Chassis.Theta + Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE),
//                                                  -180, 180);
//         }
//     }
// }

// void Find_heading_2heads_H(void)
// {
//     // Search the direction of Gimbal
//     if (Chassis.Heading < 0.01f)
//     {
//         if ((Chassis.Mode != Silence_Mode) && (Chassis.Theta < 90 + Chassis.Heading) && (Chassis.Theta > -90 + Chassis.Heading))
//         {
//             Chassis.HeadingFlag = 0;

//             Chassis.Theta = loop_float_constrain((Chassis.Theta + Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE),
//                                                  -180, 180);
//         }
//         else if (((Chassis.Mode != Silence_Mode) && (fabsf(Chassis.Theta) > 90) && (Chassis.Heading == 0)) /*|| ((Chassis.Mode != Silence_Mode) && (Chassis.Heading != 0) && (Chassis.Theta < -90+ Chassis.Heading))*/)
//         {

//             Chassis.HeadingFlag = 2;

//             Chassis.Theta = loop_float_constrain((Chassis.Theta + Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE),
//                                                  -180, 180);
//         }
//     }
//     else
//     {
//         if ((Chassis.Mode != Silence_Mode) && (Chassis.Theta < 90 + Chassis.Heading) && (Chassis.Theta > -90 + Chassis.Heading))
//         {
//             Chassis.HeadingFlag = 0;

//             Chassis.Theta = loop_float_constrain((Chassis.Theta + Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE),
//                                                  -180, 180);
//         }

//         else if ((Chassis.Mode != Silence_Mode) && (Chassis.Theta < -90 + Chassis.Heading))
//         {

//             Chassis.HeadingFlag = 2;

//             Chassis.Theta = loop_float_constrain((Chassis.Theta + Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE),
//                                                  -180, 180);
//         }
//     }
// }

// void Find_heading_2heads_L(void)
// {
//     // Search the direction of Gimbal
//     // if (Chassis.Heading < 0.01f)
//     // {
//     //     if ((Chassis.Mode != Silence_Mode) && ((Chassis.Theta > 0 + Chassis.Heading) && (Chassis.Theta < 180 + Chassis.Heading)))
//     //     {
//     //         Chassis.HeadingFlag = -1;

//     //         Chassis.Theta = loop_float_constrain((Chassis.Theta + Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE),
//     //                                              -180, 180);
//     //     }
//     //     else if ((Chassis.Mode != Silence_Mode) && (Chassis.Theta > -180 + Chassis.Heading) && (Chassis.Theta < 0 + Chassis.Heading))
//     //     {

//     //         Chassis.HeadingFlag = 1;

//     //         Chassis.Theta = loop_float_constrain((Chassis.Theta + Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE),
//     //                                              -180, 180);
//     //     }
//     // }
//     // else
//     // {
//     //     if ((Chassis.Mode != Silence_Mode) && ((Chassis.Theta > 0 + Chassis.Heading) && (Chassis.Theta < 180 + Chassis.Heading)))
//     //     {
//     //         Chassis.HeadingFlag = -1;

//     //         Chassis.Theta = loop_float_constrain((Chassis.Theta + Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE),
//     //                                              -180, 180);
//     //     }
//     //     else if ((Chassis.Mode != Silence_Mode) && (Chassis.Theta > -180 + Chassis.Heading) && (Chassis.Theta < 0 + Chassis.Heading))
//     //     {

//     //         Chassis.HeadingFlag = 1;

//     //         Chassis.Theta = loop_float_constrain((Chassis.Theta + Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE),
//     //                                              -180, 180);
//     //     }
//     // }
//     Chassis.HeadingFlag = 1;
//     Chassis.Theta = loop_float_constrain((Chassis.Theta + Chassis.HeadingFlag * YAW_REDUCTION_CORRECTION_ANGLE), -180, 180);
// }

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
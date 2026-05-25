#include "bsp_buzzer.h"
#include "tim.h" 

#define BUZZER_TIM htim4
#define BUZZER_CHANNEL TIM_CHANNEL_3
extern TIM_HandleTypeDef BUZZER_TIM;

#define DEFAULT_ON_MS 150
#define DEFAULT_OFF_MS 100
#define DEFAULT_PWM 5000  // 调节此占空比可修改音量

BuzzerController buzzer_ctrl = {0};

void buzzerInit(void)
{
    buzzer_ctrl.target_beeps_ = 0;
    buzzer_ctrl.current_beeps_ = 0;
    buzzer_ctrl.beep_timer_ = 0;
    buzzer_ctrl.on_time_ms_ = DEFAULT_ON_MS;
    buzzer_ctrl.off_time_ms_ = DEFAULT_OFF_MS;
    buzzer_ctrl.pwm_compare_ = DEFAULT_PWM;

    // 开启定时器 PWM 输出通道
    HAL_TIM_PWM_Start(&BUZZER_TIM, BUZZER_CHANNEL);
}

static void buzzerSetPwm(uint16_t pwm)
{
    __HAL_TIM_SET_COMPARE(&BUZZER_TIM, BUZZER_CHANNEL, pwm);
}

// 启动鸣叫指令，非阻塞
void buzzerStartBeep(uint8_t count)
{
    buzzer_ctrl.target_beeps_ = count;
    buzzer_ctrl.current_beeps_ = 0;
    buzzer_ctrl.beep_timer_ = 0;
}

void buzzerUpdate(void)
{
    if (buzzer_ctrl.target_beeps_ == 0)
    {
        buzzerSetPwm(0);
        return;
    }

    buzzer_ctrl.beep_timer_++;

    if (buzzer_ctrl.beep_timer_ <= buzzer_ctrl.on_time_ms_)
    {
        buzzerSetPwm(buzzer_ctrl.pwm_compare_);
    }
    else if (buzzer_ctrl.beep_timer_ <= (buzzer_ctrl.on_time_ms_ + buzzer_ctrl.off_time_ms_))
    {
        buzzerSetPwm(0);
    }
    else
    {
        buzzer_ctrl.beep_timer_ = 0;
        buzzer_ctrl.current_beeps_++;

        if (buzzer_ctrl.current_beeps_ >= buzzer_ctrl.target_beeps_)
        {
            buzzer_ctrl.target_beeps_ = 0;
        }
    }
}
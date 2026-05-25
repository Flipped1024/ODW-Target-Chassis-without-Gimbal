#ifndef __BSP_BUZZER_H
#define __BSP_BUZZER_H

#include "stdint.h"

typedef struct {
    uint8_t target_beeps_;
    uint8_t current_beeps_;
    uint16_t beep_timer_;
    uint16_t on_time_ms_;
    uint16_t off_time_ms_;
    uint16_t pwm_compare_;
} BuzzerController;

extern BuzzerController buzzer_ctrl;

void buzzerInit(void);
void buzzerStartBeep(uint8_t count);
void buzzerUpdate(void);

#endif
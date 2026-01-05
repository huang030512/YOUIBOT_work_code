#ifndef PWM_CONTROLLER_H
#define PWM_CONTROLLER_H

#include "main.h"
#include <stdbool.h>

// PWM通道结构体
typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    uint32_t current_freq;
    uint32_t max_freq;
    uint32_t min_freq;
    float duty_cycle;
    bool is_enabled;
} PWM_Controller;

// 函数声明
void PWM_Init(PWM_Controller *pwm, TIM_HandleTypeDef *htim, uint32_t channel, 
              uint32_t default_freq, uint32_t min_freq, uint32_t max_freq);
void PWM_Start(PWM_Controller *pwm);
void PWM_Stop(PWM_Controller *pwm);
void PWM_SetFrequency(PWM_Controller *pwm, uint32_t freq);
void PWM_SetDutyCycle(PWM_Controller *pwm, float duty_cycle);
uint32_t PWM_GetFrequency(PWM_Controller *pwm);
float PWM_GetDutyCycle(PWM_Controller *pwm);
bool PWM_IsEnabled(PWM_Controller *pwm);

#endif
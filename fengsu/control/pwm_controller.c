#include "pwm_controller.h"

// PWM控制器初始化
void PWM_Init(PWM_Controller *pwm, TIM_HandleTypeDef *htim, uint32_t channel, 
              uint32_t default_freq, uint32_t min_freq, uint32_t max_freq) {
    pwm->htim = htim;
    pwm->channel = channel;
    pwm->min_freq = min_freq;
    pwm->max_freq = max_freq;
    pwm->is_enabled = false;
    
    // 设置默认频率和占空比
    PWM_SetFrequency(pwm, default_freq);
    PWM_SetDutyCycle(pwm, 0.2f); // 默认30%占空比
}

// 启动PWM输出
void PWM_Start(PWM_Controller *pwm) {
    if (pwm->htim == NULL) return;
    
    HAL_TIM_PWM_Start(pwm->htim, pwm->channel);
    pwm->is_enabled = true;
}

// 停止PWM输出
void PWM_Stop(PWM_Controller *pwm) {
    if (pwm->htim == NULL) return;
    
    HAL_TIM_PWM_Stop(pwm->htim, pwm->channel);
    pwm->is_enabled = false;
}

// 设置PWM频率 (Hz)
void PWM_SetFrequency(PWM_Controller *pwm, uint32_t freq) {
    if (pwm->htim == NULL) return;
    
    // 频率范围限制
    if (freq < pwm->min_freq) freq = pwm->min_freq;
    if (freq > pwm->max_freq) freq = pwm->max_freq;
    
    // 计算定时器预分频器和周期值
    // 系统时钟频率 / (预分频器 + 1) / 自动重装载值 = PWM频率
    uint32_t tim_clock = HAL_RCC_GetPCLK1Freq(); // 获取定时器时钟频率
    if (pwm->htim->Instance == TIM1) {
        tim_clock = HAL_RCC_GetPCLK2Freq(); // TIM1在APB2总线上
    }
    
    uint32_t prescaler = 0;
    uint32_t period = 0;
    
    // 自动计算预分频器和周期值
    prescaler = (tim_clock / freq) / 7200;
    period = (tim_clock / (prescaler + 1)) / freq;
    
    // 限制范围
    if (period > 65535) period = 65535;
    if (prescaler > 65535) prescaler = 65535;
    
    // 停止PWM输出以修改配置
    bool was_enabled = pwm->is_enabled;
    if (was_enabled) {
        PWM_Stop(pwm);
    }
    
    // 更新定时器配置
    pwm->htim->Instance->PSC = prescaler;
    pwm->htim->Instance->ARR = period - 1;
    
    // 重新计算占空比以保持设置
    uint32_t pulse = (uint32_t)(period * pwm->duty_cycle);
    __HAL_TIM_SET_COMPARE(pwm->htim, pwm->channel, pulse);
    
    pwm->current_freq = freq;
    
    // 如果之前是启动状态，重新启动
    if (was_enabled) {
        PWM_Start(pwm);
    }
}

// 设置PWM占空比 (0.0 - 1.0)
void PWM_SetDutyCycle(PWM_Controller *pwm, float duty_cycle) {
    if (pwm->htim == NULL) return;
    
    // 占空比范围限制
    if (duty_cycle < 0.0f) duty_cycle = 0.0f;
    if (duty_cycle > 1.0f) duty_cycle = 1.0f;
    
    pwm->duty_cycle = duty_cycle;
    
    // 计算脉冲值
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(pwm->htim);
    uint32_t pulse = (uint32_t)(period * duty_cycle);
    
    __HAL_TIM_SET_COMPARE(pwm->htim, pwm->channel, pulse);
}


// 获取当前频率
uint32_t PWM_GetFrequency(PWM_Controller *pwm) {
    return pwm->current_freq;
}

// 获取当前占空比
float PWM_GetDutyCycle(PWM_Controller *pwm) {
    return pwm->duty_cycle;
}

// 检查PWM是否启用
bool PWM_IsEnabled(PWM_Controller *pwm) {
    return pwm->is_enabled;
}
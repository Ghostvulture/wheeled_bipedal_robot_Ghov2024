#include "bsp_pwm.h"

extern TIM_HandleTypeDef htim5;

void PWM_Init(void)
{
    HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_3);
}

void PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel)
{
    HAL_TIM_PWM_Start(htim, Channel);
}

void PWM_Stop(TIM_HandleTypeDef *htim, uint32_t Channel)
{
    HAL_TIM_PWM_Stop(htim, Channel);
}

void PWM_SetPeriod(TIM_HandleTypeDef *htim, float period)
{
    __HAL_TIM_SetAutoreload(htim, period * ((84000000) / (htim->Init.Prescaler + 1)));
}

void PWM_SetDutyRatio(TIM_HandleTypeDef *htim, float dutyratio, uint32_t channel)
{
    __HAL_TIM_SetCompare(htim, channel, dutyratio * (htim->Instance->ARR));
}

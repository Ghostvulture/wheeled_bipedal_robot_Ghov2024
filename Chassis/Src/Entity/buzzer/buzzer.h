#ifndef BUZZER_H
#define BUZZER_H

#include "main.h"
#include "bsp_pwm.h"

extern TIM_HandleTypeDef htim4;

#define BUZZER_TIM htim4
#define BUZZER_CHANNEL TIM_CHANNEL_3

/*-----------------------------------------------音调-----------------------------------------------*/
#define DoFreq 523
#define ReFreq 587
#define MiFreq 659
#define FaFreq 698
#define SoFreq 784
#define LaFreq 880
#define SiFreq 988

/**
 * @brief  蜂鸣器初始化
 */
void Buzzer_Init(void);

/**
 * @brief  蜂鸣器开始响
 * @param  freq: 频率
 * @param  volume: 音量
 * @todo  音量未实现，设置占空比好像没有用
 */
void Buzzer_Start(float freq, float volume);

/**
 * @brief  蜂鸣器停止响
 */
void Buzzer_Stop(void);

#endif // !BUZZER_H

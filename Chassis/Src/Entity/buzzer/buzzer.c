#include "buzzer.h"

void Buzzer_Init(void)
{
    PWM_SetPeriod(&BUZZER_TIM, 1.0 / 1000);
    PWM_SetDutyRatio(&BUZZER_TIM, 0.5, BUZZER_CHANNEL);
    PWM_Stop(&BUZZER_TIM, BUZZER_CHANNEL);
}

void Buzzer_Start(float freq, float volume)
{
    PWM_SetPeriod(&BUZZER_TIM, 1.0 / freq);
    PWM_SetDutyRatio(&BUZZER_TIM, volume, BUZZER_CHANNEL);
    PWM_Start(&BUZZER_TIM, BUZZER_CHANNEL);
}

void Buzzer_Stop(void)
{
    PWM_Stop(&BUZZER_TIM, BUZZER_CHANNEL);
}

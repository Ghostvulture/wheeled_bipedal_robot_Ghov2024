# PWM

和TIM重复比较大，基本封装了以下几个函数：
void PWM_Init(void);
void PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
void PWM_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);
void PWM_SetPeriod(TIM_HandleTypeDef *htim, float period);
void PWM_SetDutyRatio(TIM_HandleTypeDef *htim, float dutyratio, uint32_t channel);
#include "bsp_tim.h"
#include "stm32f4xx_it.h"
#include "AHRS.hpp"

/**
 * @brief 这是hal库的定时器中断函数
 * @attention 每次用MX生成代码时都会在stm32f4xx_it文件中生成这个函数，这里将这个函数的定义放在这里以便管理和开发，因此每次MX重新生成代码时需要将stm32f4xx_it中这个函数的定义注释掉
 */

extern TIM_HandleTypeDef htim2;

void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
	AHRS::instance()->AHRS_Update();
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

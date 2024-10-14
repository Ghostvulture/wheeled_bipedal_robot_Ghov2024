#include "bsp_gpio.h"
#include "cpp_solution.hpp"
#include "IST8310.hpp"

void gpio_init(void)
{
    ///< 对LED引脚进行初始化配置
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, GPIO_PIN_RESET);

    ///< 对spi片选线进行初始化
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
}

/**
 * @brief 这是对HAL库中的弱定义的GPIO中断回调函数的重写
 * 在这里我们对IST8310的数据进行更新
*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == IST8310_DRDY_GPIOp)
    {
        ist8310_getMagData();
    }
}

#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#include "main.h"

#ifdef __cplusplus
extern "C"
{
#endif
void gpio_init(void);

/**
 * @brief  获取指定GPIO口的电平状态
 * @param  
 * @return GPIO_PIN_SET / GPIO_PIN_RESET
*/
//GPIO_PinState gpio_pin_read(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

#ifdef __cplusplus
}
#endif
#endif // _GPIO_H

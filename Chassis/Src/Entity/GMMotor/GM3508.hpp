#ifndef GM3508_HPP
#define GM3508_HPP

// #include "stm32f4xx_hal.h"
// #include "stm32f4xx_ll_rcc.h"
// #include "stm32f4xx_ll_bus.h"
// #include "stm32f4xx_ll_system.h"
// #include "stm32f4xx_ll_exti.h"
// #include "stm32f4xx_ll_cortex.h"
// #include "stm32f4xx_ll_utils.h"
// #include "stm32f4xx_ll_pwr.h"
// #include "stm32f4xx_ll_dma.h"
// #include "stm32f4xx_ll_gpio.h"
#include "main.h"
#include "GMMotor.hpp"

/**
 * @class GM3508
 * @brief GM3508电机的控制类，继承自Motor类。
 *
 * 这个类实现了GM3508电机的控制方法，包括输出设置等。
 */
class GM3508 : public GMMotor
{
public:
    /**
     * @brief 构造函数，初始化GM6020电机控制类。
     */
    GM3508();

    /**
     * @brief 构造函数，初始化GM6020电机控制类。
     */
    ~GM3508();

    /**
     * @brief 实现电机输出设置。
     * 根据当前的控制模式和PID反馈调整电机输出。
     * 必须在派生类中具体实现此方法以适应具体电机。
     */
    void setOutput() override;
};

#endif // GM3508_HPP

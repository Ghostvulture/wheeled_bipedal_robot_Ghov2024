#ifndef GM2006_HPP
#define GM2006_HPP

#include "main.h"
#include "GMMotor.hpp"

/**
 * @class GM2006
 * @brief GM2006电机的控制类，继承自Motor类。
 *
 * 这个类实现了GM2006电机的控制方法，包括输出设置等。
 */
class GM2006 : public GMMotor
{
public:
    /**
     * @brief 构造函数，初始化GM6020电机控制类。
     */
    GM2006();

    /**
     * @brief 构造函数，初始化GM6020电机控制类。
     */
    ~GM2006();

    /**
     * @brief 实现电机输出设置。
     * 根据当前的控制模式和PID反馈调整电机输出。
     * 必须在派生类中具体实现此方法以适应具体电机。
     */
    void setOutput() override;
};

#endif // GM2006_HPP

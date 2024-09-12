
#ifndef ODOMETER_HPP
#define ODOMETER_HPP

#include "main.h"
#include "LKMotor.hpp"

class Odometer
{
private:
    /**
     * @brief 里程计数据
     */
    uint16_t wheel_last_ecd;
    uint16_t wheel_ecd;
    uint16_t ecd_error;

    //传入指针
    LKMotor *leftMotor;
    LKMotor *rightMotor;

public:
    /**
     * @brief 构造函数
     */
    Odometer(LKMotor& lMotor, LKMotor& rMotor) : leftMotor(lMotor), rightMotor(rMotor)
    {}

    /**
     * @brief 析构函数
     */
    ~Odometer()
    {}

    /**
     * @brief 里程计初始化
     */
    void init()
    {
        wheel_last_ecd = leftMotor.motorFeedback.positionFdb; // 初始化为左轮位置反馈
    }

    /**
     * @brief 里程计更新并根据里程计数据计算相对位移
     */
    uint16_t update()
    {
        wheel_ecd = leftMotor.motorFeedback.positionFdb; // 获取当前编码器数据
        ecd_error = wheel_ecd - wheel_last_ecd;          // 计算编码器增量
        wheel_last_ecd = wheel_ecd;                      // 更新最后的编码器位置
    }

    /**
     * @brief 里程计重置
     */
    void clear()
    {
        wheel_last_ecd = 0;
        wheel_ecd = 0;
        ecd_error = 0;
    }


    /**
     * @brief 根据里程计数据计算相对位移（以0为set，此作为fdb）
     */
    uint16_t calculateRelativeDisplacement()
    {
        wheel_last_ecd = 
        ecd_error = wheel_ecd - wheel_last_ecd;
    }

};

#endif // ODOMETER_HPP

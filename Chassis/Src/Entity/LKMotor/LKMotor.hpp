#ifndef LKMOTOR_HPP
#define LKMOTOR_HPP

#include "Pid.hpp"
#include "main.h"

/**
 * @class LKMotor
 * @brief 电机控制类，提供电机的基本控制功能。
 *
 * 该类实现了电机的各种控制模式，包括速度、位置和基于IMU的控制。
 * 它还负责处理电机反馈数据和执行PID控制。
 */
class LKMotor
{
public:
    /**
     * @brief 定义电机测量数据的结构体。
     * @param last_ecd 上次电机编码器的读数。
     * @param ecd 当前电机编码器的读数。
     * @param speed_dps 电机的转速，单位为rpm。
     * @param given_current 给定的电机电流，单位为毫安。
     * @param temperate 电机的温度，单位为摄氏度。
     */
    typedef struct
    {
        int16_t last_ecd;      ///< 上次电机编码器的读数
        uint16_t ecd;          ///< 当前电机编码器的读数
        int16_t speed_dps;     ///< 电机的转速，单位rpm
        int16_t given_current; ///< 给定的电机电流
        uint8_t temperate;     ///< 电机的温度
        
        uint8_t torqueFdb;     ///< 电机的转矩反馈
    } motor_measure_t;

    /**
     * @enum MotorControlModeType
     * @brief 描述电机的不同控制模式。
     */
    enum MotorControlModeType
    {
        RELAX_MODE,          ///< 电机松开模式，所有输出均为0。
        SPD_MODE,            ///< 速度模式，控制电机速度。
        POS_MODE,            ///< 位置模式，控制电机到特定位置。
        POS_FOR_NO_SPD_MODE, ///< 无速度反馈下的位置模式。
        IMU_MODE             ///< IMU模式，使用IMU反馈进行控制。
    };

    /**
     * @struct MotorFeedBack
     * @brief 电机反馈数据的结构体，包括电机的各种物理量反馈。
     * 结构体中包含了电机的电流、速度、位置等信息，以及电机的温度等状态反馈。
     */
    struct MotorFeedBack
    {
        int16_t currentFdb;    ///< 电机电流反馈
        float speedFdb;        ///< 电机当前速度反馈, 单位rad/s
        float lastSpeedFdb;    ///< 上次记录的电机速度
        float positionFdb;     ///< 电机当前位置反馈
        float lastPositionFdb; ///< 上次记录的电机位置
        float temperatureFdb;  ///< 电机温度反馈

        float torqueFdb;       ///< 电机转矩反馈

        float ecd;             ///< 电机编码器值
        float last_ecd;         ///< 上次电机编码器值
    };

    MotorControlModeType controlMode; ///< 当前电机控制模式
    MotorFeedBack motorFeedback;      ///< 电机的反馈数据
    uint16_t canId;                   ///< 电机的CAN通信ID
    CAN_HandleTypeDef *hcan;          ///< 指向电机使用的CAN接口的指针

    Pid speedPid;    ///< 速度环PID控制器
    Pid positionPid; ///< 位置环PID控制器
    Pid anglePid;    ///< 角度环PID控制器
    Pid omegaPid;   ///< 角速度环PID控制器

    float speedSet;    ///< 设定的目标速度
    float positionSet; ///< 设定的目标位置，范围[-Pi, Pi]
    float angleSet;    ///< 设定的目标角度

    float offset;     ///< 电机的初始位置偏移
    
    int16_t currentSet;  ///< 设定的电流输出
    uint16_t maxCurrent; ///< 最大电流限制

    float torqueSet; ///< 设定的目标转矩

    /**
     * @brief 纯虚函数，用于设置电机输出。
     * 必须在派生类中实现此函数。
     */
    virtual void setOutput() = 0;

    /**
     * @brief 构造函数
     */
    LKMotor()
    {
        controlMode = RELAX_MODE;

        speedSet = 0;
        positionSet = 0;
        currentSet = 0;

        maxCurrent = 0;

        motorFeedback.speedFdb = 0;
        motorFeedback.lastSpeedFdb = 0;
        motorFeedback.positionFdb = 0;
        motorFeedback.lastPositionFdb = 0;
        motorFeedback.temperatureFdb = 0;

        // pid初始化
        speedPid.mode = Pid::PID_POSITION;
        speedPid.kp = 0.1;
        speedPid.ki = 0.0;
        speedPid.kd = 0.0;
        speedPid.maxOut = 25000;
        speedPid.maxIOut = 3;

        positionPid.mode = Pid::PID_POSITION;
        positionPid.kp = 0.1;
        positionPid.ki = 0.0;
        positionPid.kd = 0.0;
        positionPid.maxOut = 25000;
        positionPid.maxIOut = 3;

        anglePid.mode = Pid::PID_POSITION;
        anglePid.kp = 0.1;
        anglePid.ki = 0.0;
        anglePid.kd = 0.0;
        anglePid.maxOut = 25000;
        anglePid.maxIOut = 3;
    }
};

#endif // LKMOTOR_HPP

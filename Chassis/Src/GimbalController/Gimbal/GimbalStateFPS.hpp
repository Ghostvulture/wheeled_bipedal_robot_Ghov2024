#ifndef GIMBAL_STATE_FPS_HPP
#define GIMBAL_STATE_FPS_HPP

#include "StateMachine.hpp"

#include "GMMotorHandler.hpp"
#include "BoardConnectivity.hpp"
#include "Dr16.hpp"
#include "GM6020.hpp"
#include "GM3508.hpp"
#include "GM2006.hpp"
#include "Pid.hpp"
#include "AHRS.hpp"

#include "Math.hpp"
#include "string.h"
#include "FirstOrderFilter.hpp"

/**
 * @class GimbalStateRelax
 * @brief 底盘跟随状态机
 * @note 云台放松状态机继承自状态机基类。
 */
class GimbalStateFPS : public StateMachine
{
public:
    /**
     * @brief 构造函数。
     * @param YawMotor 云台Yaw轴电机。
     * @param PitchMotor 云台Pitch轴电机。
     */
    GimbalStateFPS(GM6020 *YawMotor,
                   GM6020 *PitchMotor) : YawMotor(YawMotor),
                                         PitchMotor(PitchMotor) {};

    /**
     * @brief 析构函数。
     */
    ~GimbalStateFPS() {};

    /**
     * @brief 云台Yaw轴电机。
     */
    GM6020 *YawMotor;

    /**
     * @brief Yaw轴电机位置设置的一阶滤波器。
     */
    FirstOrderFilter YawSetFilter;

    /**
     * @brief Yaw轴电机位置反馈的一阶滤波器。
     */
    FirstOrderFilter YawFdbFilter;

    /**
     * @brief 云台Pitch轴电机。
     */
    GM6020 *PitchMotor;

    /**
     * @brief Pitch轴电机位置设置的一阶滤波器。
     */
    FirstOrderFilter PitchSetFilter;

    /**
     * @brief Yaw轴电机初始位置。
     * @note 弧度制，[0, 2π]
     * 这个值会在init函数中被赋值，在新车下载代码时，需要先用relaxmode进行debug查看。
     */
    float YawOffset;

    /**
     * @brief Pitch轴电机初始位置。
     * @note 弧度制，[0, 2π]
     * 这个值会在init函数中被赋值，需要根据实际情况修改这个值。会在init函数中被赋值，在新车下载代码时，需要先用relaxmode进行debug查看。
     */
    float PitchOffset;

    /**
     * @brief Yaw轴电机的位置或者速度设定。
     * 并且还需要根据实际情况添加限幅
     */
    float YawSet;

    /**
     * @brief Pitch轴电机的位置或者速度设定。
     * 并且还需要根据实际情况添加限幅
     */
    float PitchSet;

    /**
     * @brief 云台上限
     */
    float PitchUpperLimit;

    /**
     * @brief 云台下限
     */
    float PitchLowerLimit;

    /**
     * @brief 云台Yaw轴电机的前馈控制量。从底盘状态机中获取，目前是直接从chassiscontroller中获取。
     * @todo 从底盘C板发送回来的数据中获取。
     */
    float ForwardVw;
    
    /**
     * @brief 初始化各种参数。
     */
    void init() override;

    /**
     * @brief 进入放松状态。
     * 将电机设置为RELAX_MODE。
     */
    void enter() override;

    /**
     * @brief 执行放松状态。
     * setOutput()函数设置电机的输出。
     * 电机输出为0。
     */

    void execute() override;

    /**
     * @brief 退出放松状态。
     */
    void exit() override;

    /**
     * @brief 运行放松状态。
     * enter() -> execute() -> exit()
     */
    void run() override;
};

#endif

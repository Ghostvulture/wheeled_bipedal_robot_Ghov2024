#ifndef GIMBAL_STATE_REMOTE_CONTROL_HPP
#define GIMBAL_STATE_REMOTE_CONTROL_HPP

#include "StateMachine.hpp"

#include "GMMotorHandler.hpp"
#include "Dr16.hpp"
#include "GM6020.hpp"
#include "GM3508.hpp"
#include "GM2006.hpp"
#include "AHRS.hpp"

#include "Math.hpp"

/**
 * @class GimbalStateRemoteControl
 * @brief 云台遥控状态机。
 * 这个模式一般不使用 
 */
class GimbalStateRemoteControl : public StateMachine
{
public:
    /**
     * @brief 构造函数。
     * @param YawMotor 云台Yaw轴电机。
     * @param PitchMotor 云台Pitch轴电机。
     */
    GimbalStateRemoteControl(GM6020 *YawMotor,
                             GM6020 *PitchMotor) : YawMotor(YawMotor),
                                                   PitchMotor(PitchMotor){};

    /**
     * @brief 析构函数。
     */
    ~GimbalStateRemoteControl(){};

    /**
     * @brief 云台Yaw轴电机。
     */
    GM6020 *YawMotor;

    /**
     * @brief 云台Pitch轴电机。
     */
    GM6020 *PitchMotor;

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
     * @brief Yaw轴电机的位置设定。
     * 单位为弧度，范围为[-∞, ∞]。
     * 最后会在execute函数中映射到[0, 2π]。
     * 并且还需要根据实际情况添加限幅
     */
    float YawSet;

    /**
     * @brief Pitch轴电机的位置设定。
     * 单位为弧度，范围为[-∞, ∞]。
     * 最后会在execute函数中映射到[0, 2π]。
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
     * @brief 初始化遥控状态。
     * 将电机设置为SPD_MODE。并设置电机的PID参数。
     */
    void init() override;

    /**
     * @brief 进入遥控状态。
     * 接受遥控器输入。
     */
    void enter() override;

    /**
     * @brief 执行放松状态。
     * 处理电机输出设定。
     */
    void execute() override;

    /**
     * @brief 退出状态。
     */
    void exit() override;

    /**
     * @brief 运行状态。
     * enter -> execute -> exit
     */
    void run() override;
};

#endif

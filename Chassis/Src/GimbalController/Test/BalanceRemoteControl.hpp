#ifndef BALANCE_REMOTE_CONTROL_HPP
#define BALANCE_REMOTE_CONTROL_HPP

#include "StateMachine.hpp"

#include "GMMotorHandler.hpp"
#include "BoardConnectivity.hpp"
#include "Dr16.hpp"
#include "GM6020.hpp"
#include "GM3508.hpp"
#include "GM2006.hpp"
#include "LK9025.hpp"
#include "LK8016.hpp"
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
class BalanceRemoteControl : public StateMachine
{
public:
    /**
     * @brief 构造函数。
     * @param YawMotor 云台Yaw轴电机。
     * @param PitchMotor 云台Pitch轴电机。
     */
    BalanceRemoteControl(LK9025 *LMotor,
                         LK9025 *RMotor,
                         LK8016 *RD) : LMotor(LMotor),
                                       RMotor(RMotor),
                                       RD(RD) {};

    /**
     * @brief 析构函数。
     */
    ~BalanceRemoteControl() {};

    /**
     * @brief 电机。
     */
    LK9025 *LMotor;
    LK9025 *RMotor;
    LK8016 *RD;

    /**
     * @brief pid结构体。
     */
    Pid LK9025SpeedPid;///< 速度环PID
    Pid LK9025PositionPid;///< 位置环PID
    Pid LK8016PositionPid;///< 位置环PID

    /**
     * @brief Yaw轴电机的位置或者速度设定。
     * 并且还需要根据实际情况添加限幅
     */
    float Motor_PositionSet;

    
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

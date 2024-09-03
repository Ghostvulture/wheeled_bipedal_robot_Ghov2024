#ifndef BALANCE_SPEED_CONTROL_HPP
#define BALANCE_SPEED_CONTROL_HPP

#include "StateMachine.hpp"

#include "GMMotorHandler.hpp"
#include "Dr16.hpp"
#include "GM6020.hpp"
#include "GM3508.hpp"
#include "GM2006.hpp"
#include "AHRS.hpp"
#include "LK9025.hpp"

#include "Math.hpp"

/**
 * @class GimbalStateRemoteControl
 * @brief 云台遥控状态机。
 * 这个模式一般不使用 
 */
class BalanceSpeedControl : public StateMachine
{
public:
    /**
     * @brief 构造函数。
     */
    BalanceSpeedControl(LK9025 *LMotor,
                         LK9025 *RMotor) : LMotor(LMotor),
                                           RMotor(RMotor) {};

    /**
     * @brief 析构函数。
     */
    ~BalanceSpeedControl(){};

    /**
     * @brief test
     */
    LK9025 *LMotor;
    LK9025 *RMotor;

    /**
     * @brief pid结构体。
     */
    Pid LK9025SpeedPid;///< 速度环PID

    /**
     * @brief 电机的位置设定。
     */
    float Motor_SpeedSet;


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

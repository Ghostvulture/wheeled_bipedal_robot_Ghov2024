#ifndef BALANCE_STATE_RELAX_HPP
#define BALANCE_STATE_RELAX_HPP

#include "StateMachine.hpp"

#include "GMMotorHandler.hpp"
#include "Dr16.hpp"
#include "GM6020.hpp"
#include "GM3508.hpp"
#include "GM2006.hpp"
#include "LK9025.hpp"

/**
 * @class GimbalStateRelax
 * @brief 云台放松状态机，电机不动。
 * @note 云台放松状态机继承自状态机基类。
 */
class BalanceStateRelax : public StateMachine
{
public:
    /**
     * @brief 构造函数，传入云台Yaw轴电机和Pitch轴电机。
     * @param YawMotor 云台Yaw轴电机。
     * @param PitchMotor 云台Pitch轴电机。
     */
    BalanceStateRelax(LK9025 *LMotor,
                      LK9025 *RMotor) : LMotor(LMotor),
                                        RMotor(RMotor) {};

    /**
     * @brief 析构函数。
     */
    ~BalanceStateRelax(){};

    /**
     * @brief 云台Yaw轴电机。
     */
    LK9025 *LMotor;
    LK9025 *RMotor;


    /**
     * @brief 初始化放松状态。
     * 清空云台电机的PID。
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

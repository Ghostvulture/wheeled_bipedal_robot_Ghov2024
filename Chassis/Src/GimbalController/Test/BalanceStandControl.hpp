#ifndef BALANCE_STAND_CONTROL_HPP
#define BALANCE_STAND_CONTROL_HPP

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
 * @class BalanceStandControl
 * @brief pid平衡控制状态机。
 */
class BalanceStandControl : public StateMachine
{
public:
    /**
     * @brief 构造函数。
     */
    BalanceStandControl(LK9025 *LMotor,
                         LK9025 *RMotor) : LMotor(LMotor),
                                           RMotor(RMotor) {};

    /**
     * @brief 析构函数。
     */
    ~BalanceStandControl(){};

    /**
     * @brief test
     */
    LK9025 *LMotor;
    LK9025 *RMotor;

    /**
     * @brief pid结构体。
     */
    Pid LK9025AnglePid;///< 速度环PID
    Pid LK9025OmegaPid;///< 位置环PID
    Pid LK9025PositionPid;///< 位置环PID

    /**
     * @brief 电机的位置设定。
     * @param Motor_angleSet 电机的角度设定。
     * @param Motor_omegaSet 电机的角速度设定。
     */
    float Motor_angleSet;
    float Motor_omegaSet;
    float Motor_positionSet;

    /**
     * @brief 电机的位置获取。
     * @param Motor_angleRef 电机的角度设定。
     * @param Motor_omegaRef 电机的角速度设定。
     */
    float Motor_angleRef;
    float Motor_omegaRef;
    float Motor_positionRef;


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

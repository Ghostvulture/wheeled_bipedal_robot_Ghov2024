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

#include "msgs.h"
#include "balance_task.hpp"
#include "kalman_filter.h"

#include "arm_math.h"

#define WHEEL_RADIUS 0.095f

/**
 * @class BalanceStandControl
 * @brief pid平衡控制状态机。
 */
class BalanceStandControl : public StateMachine
{
public:
    /**
     * @brief 构造函数。
     * @note (Type arg) 构造函数的参数列表，用于传递外部的数据或对象给类的成员变量
     * @note member(arg) 初始化列表，用于在进入构造函数体之前对成员变量进行初始化
     */
    BalanceStandControl(LK9025 *LMotor,
                         LK9025 *RMotor) : LMotor(LMotor),
                                           RMotor(RMotor){};

    /**
     * @brief 析构函数。
     */
    ~BalanceStandControl(){};

    /**
     * @brief test
     */
    LK9025 *LMotor;
    LK9025 *RMotor;
    Msg_Odometer_t odometer_msg;
    cVelFusionKF kf_vel;

    /**
     * @brief 遥控器
     */
    float Vx;

    /**
     * @brief pid结构体。
     */
    Pid LK9025AnglePid;///< 速度环PID
    Pid LK9025OmegaPid;///< 位置环PID
    Pid LK9025PositionPid;///< 位置环PID
    Pid LK9025SpeedPid;///< 速度环PID

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
    float Motor_DisplacementRef;


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

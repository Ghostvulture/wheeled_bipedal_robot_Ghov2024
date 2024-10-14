#ifndef CHASSIS_STATE_RELAX_HPP
#define CHASSIS_STATE_RELAX_HPP

#include "StateMachine.hpp"
#include "BoardConnectivity.hpp"
#include "GMMotor.hpp"

#ifdef __cplusplus
/**
 * @class ChassisStateRelax
 * @brief 底盘放松状态机。
 */
class ChassisStateRelax : public StateMachine
{
public:

    float Vx = 0; ///< 底盘X轴速度
    float Vy = 0; ///< 底盘Y轴速度
    float Vw = 0; ///< 底盘旋转速度

    /**
     * @brief 初始化函数，设置电机数据为0
     */
    void init() override;

    /**
     * @brief 进入函数
     */
    void enter() override;

    /**
     * @brief 执行函数，设置电机速度为0
     */
    void execute() override;

    /**
     * @brief 退出函数
     */
    void exit() override;

    /**
     * @brief 运行函数，依次执行enter、execute、exit函数
     */
    void run() override;

    /**
     * @brief 构造函数
     */
    ChassisStateRelax() {}
};
#endif
#endif // CHASSIS_CONTROLLER_HPP

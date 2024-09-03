#ifndef CHASSIS_STATE_FPS_CONTROL_HPP
#define CHASSIS_STATE_FPS_CONTROL_HPP

#include "StateMachine.hpp"
#include "Dr16.hpp"
#include "BoardConnectivity.hpp"
#include "GMMotor.hpp"
#include "Pid.hpp"
#include "FirstOrderFilter.hpp"

/**
 * @class ChassisStateRemoteControl
 * @brief 底盘遥控状态机。
 * @todo 添加键盘控制逻辑
 */
class ChassisStateFPS : public StateMachine
{
public:
    float Vx;                  ///< 横向移动速度
    float Vy;                  ///< 前后移动速度
    float Vw;                  ///< 旋转速度
    Pid ChassisYawSpeedPid;    ///< 底盘跟随云台Yaw轴速度PID
    FirstOrderFilter VxFilter; ///< 横向速度滤波器
    FirstOrderFilter VyFilter; ///< 纵向速度滤波器
    FirstOrderFilter VwFilter; ///< 旋转速度滤波器
    float relativeAngle;       ///< 云台相对于底盘的角度，用于底盘跟随云台
    
    /**
     * @brief 初始化函数，设置电机数据为0
     */
    void init() override;

    /**
     * @brief 进入函数，接受遥控器指令
     */
    void enter() override;

    /**
     * @brief 执行函数，将速度指令放入BoardConnectivity的发送缓冲区
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
    ChassisStateFPS() {};
};

#endif

#ifndef CHASSIS_STATE_ROTATE_HPP
#define CHASSIS_STATE_ROTATE_HPP

#include "StateMachine.hpp"
#include "BoardConnectivity.hpp"
#include "Dr16.hpp"
#include "GMMotor.hpp"

/**
 * @class ChassisStateRotate
 * @brief 底盘小陀螺状态机。
 * @todo 添加键盘控制逻辑
 * @todo 添加小陀螺随机速度
 */
class ChassisStateRotate : public StateMachine
{
public:
    /*用于速度解算的变量，单位为 m/s。这些变量也是直接用于操控底盘移动的变量。*/
    float Vx; ///< 横向移动速度
    float Vy; ///< 前后移动速度
    float Vw; ///< 旋转速度

    /**
     * @brief 初始化函数，设置电机数据为0, w设定为一个固定值
     */
    void init() override;

    /**
     * @brief 进入函数，接收底盘速度指令，进行速度解算
     */
    void enter() override;

    /**
     * @brief 执行函数，将速度指令转化为电机速度
     */
    void execute() override;

    /**
     * @brief 退出函数，设置电机速度
     */
    void exit() override;

    /**
     * @brief 运行函数，依次执行enter、execute、exit函数
     */
    void run() override;

    ChassisStateRotate() {}
};

#endif // CHASSIS_CONTROLLER_HPP

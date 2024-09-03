#ifndef CHASSIS_CONTROLLER_HPP
#define CHASSIS_CONTROLLER_HPP

#include "Controller.hpp"

#include "ChassisStateRemoteControl.hpp"
#include "ChassisStateRelax.hpp"
#include "ChassisStateRotate.hpp"
#include "ChassisStateFPS.hpp"

#include "GMMotorHandler.hpp"
#include "Dr16.hpp"

/**
 * @struct Chassis_error_t
 * @brief 底盘错误结构体。
 * @param MOTOR_OFFLINE 电机离线。
 * @param MOTOR_TEMP_HIGH 电机温度过高。
 * @todo 需要在daemon中添加对应的错误处理函数。
 */
struct Chassis_error_t
{
    bool MOTOR_OFFLINE = false;
    bool MOTOR_TEMP_HIGH = false;
};

/**
 * @class ChassisController
 * @brief 底盘控制类，提供底盘的基本控制功能。
 * 在云台的程序中，这个类主要负责将处理好的数据发送给底盘控制板，地盘控制板会进一步处理。
 */
class ChassisController : public Controller
{
public:
    /*-----------------------------状态机-----------------------------*/

    /**
     * @brief 底盘遥控状态。
     */
    ChassisStateRemoteControl chassisStateRemoteControl;

    /**
     * @brief 底盘放松状态。
     */
    ChassisStateRelax chassisStateRelax;

    /**
     * @brief 底盘小陀螺。
     */
    ChassisStateRotate chassisStateRotate;

    /**
     * @brief 底盘跟随云台。
     */
    ChassisStateFPS chassisStateFPS;

    /**
     * @brief 底盘状态枚举。
     * @note 如果需要添加新的底盘状态，需要在此处添加。
     * @param STATE_REMOTE_CONTROL 遥控模式。
     * @param STATE_RELAX 放松模式。
     * @param STATE_ROTATE 小陀螺模式。
     */
    typedef enum ChassisState
    {
        STATE_REMOTE_CONTROL = 0,
        STATE_RELAX,
        STATE_ROTATE,
        STATE_FPS
    };

    /**
     * @brief 当前底盘状态。
     */
    ChassisState CurrentSate;

    /**
     * @brief 构造函数。
     */
    ChassisController()
    {
        CurrentSate = STATE_RELAX;
        init();
    }
    /**
     * @brief 析构函数。
     */
    ~ChassisController()
    {
    }

    /**
     * @brief 初始化函数。
     * @note 将电机遥控数据加入到发送缓冲区。全部初始化为0。
     */
    void init();

    /**
     * @brief 运行函数。
     * 根据遥控器的状态，选择不同的底盘状态。
     */
    void run() override;

    /**
     * @brief 切换底盘状态。
     * 用于简化底盘状态的切换。
     */
    void ChangeState(StateMachine state)
    {
    }

    /**
     * @brief 获取底盘控制类的单例。
     * @return ChassisController* 返回底盘控制类的单例。
     */
    static ChassisController *instance()
    {
        static ChassisController instance;
        return &instance;
    }
};

#endif // CHASSIS_CONTROLLER_HPP

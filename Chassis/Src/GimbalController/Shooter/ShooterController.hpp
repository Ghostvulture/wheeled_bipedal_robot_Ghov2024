#ifndef SHOOTER_CONTROLLER_HPP
#define SHOOTER_CONTROLLER_HPP

#include "main.h"

#include "Controller.hpp"

#include "GMMotorHandler.hpp"
#include "Dr16.hpp"
#include "GM6020.hpp"
#include "GM3508.hpp"
#include "GM2006.hpp"

/**
 * @class ShooterController
 * @brief 射击控制类。
 * @note 该类继承自Controller类。
 * @todo 考虑是否需要像云台控制类或者云台状态类一样，将射击控制类的状态进行拆分成不同的类
 */
class ShooterController : public Controller
{
public:
    /**
     * @brief 左摩擦轮电机
     */
    GM3508 LeftFricMotor;

    /**
     * @brief 右摩擦轮电机
     */
    GM3508 RightFricMotor;

    /**
     * @brief 播弹盘电机
     */
    GM2006 TriggerMotor;

    /**
     * @brief 构造函数。
     * @note 在构造函数中，将状态设置为STATE_RELAX。
     */
    ShooterController()
    {
        CurrentSate = STATE_RELAX;
    }

    /**
     * @brief 析构函数。
     */
    ~ShooterController(){};

    /**
     * @brief 发射机构状态枚举。
     * @note 如果需要添加新的发射机构状态，需要在此处添加。

     */
    typedef enum ShooterState
    {
        STATE_RELAX = 0,
        STATE_PREHEAT,
        STATE_FIRE
    };

    /**
     * @brief 当前发射机构状态。
     * 用于判断状态切换
     */
    ShooterState CurrentSate;

    /**
     * @brief 初始化函数。
     */
    void init();

    /**
     * @brief 运行函数。
     * 根据遥控器的状态，选择不同的底盘状态。
     */
    void run() override;

    /**
     * @brief 获取底盘控制类的单例。
     * @return ChassisController* 返回底盘控制类的单例。
     */
    static ShooterController *instance()
    {
        static ShooterController instance;
        return &instance;
    }
};

#endif

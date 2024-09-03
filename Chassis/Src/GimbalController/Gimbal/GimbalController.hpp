#ifndef GIMBALCONTROLLER_HPP
#define GIMBALCONTROLLER_HPP

#include "main.h"

#include "Controller.hpp"

#include "GMMotorHandler.hpp"
#include "Dr16.hpp"
#include "GM6020.hpp"
#include "GM3508.hpp"
#include "GM2006.hpp"

#include "GimbalStateRemoteControl.hpp"
#include "GimbalStateSearch.hpp"
#include "GimbalStateRelax.hpp"
#include "GimbalStateFPS.hpp"

/**
 * @struct USBSendPacket
 * @brief USB发送数据包结构体。
 * 一共20个字节。
 */
struct USBSendPacket
{
    uint8_t header = 0x5A;    // 发送数据包的头
    uint8_t detect_color : 1; // 检测到的颜色
    bool reset_tracker : 1;   // 是否重置追踪
    uint8_t reserved : 6;     // 保留位
    float q1;                 // 四元数
    float q2;
    float q3;
    float q4;
    uint16_t checksum = 0; // 校验和
} __attribute__((packed));

/**
 * @struct USBReceivePacket
 * @brief USB接收数据包结构体。
 * 一共12个字节。
 */
struct USBReceivePacket
{
    uint8_t header = 0x00; // 发送数据包的头

    bool tracking : 1;    // 跟踪的颜色
    uint8_t id : 3;       // 识别的id
    uint8_t reserved : 4; // 保留位

    float pitch; // 俯仰角
    float yaw;   // 偏航角

    uint16_t checksum = 0; // 校验和
} __attribute__((packed));

/**
 * @class GibmalController
 * @brief 云台控制类，主要操控云台的电机。
 */
class GimbalController : public Controller
{
public:
    /**
     * @brief 云台Yaw轴电机。
     * 电机实例，将会传入状态机中
     */
    GM6020 YawMotor;

    /**
     * @brief 云台Pitch轴电机。
     * 电机实例，将会传入状态机中
     */
    GM6020 PitchMotor;

    /**
     * @brief 云台状态枚举。
     * @note 如果需要添加新的状态，需要在此处添加。
     * @param STATE_REMOTE_CONTROL 遥控模式。
     * @param STATE_SEARCH 自瞄模式。
     * @param STATE_RELAX 放松模式。
     */
    enum GimbalState
    {
        STATE_REMOTE_CONTROL = 0,
        STATE_SEARCH,
        STATE_RELAX,
        STATE_FPS
    };

    /**
     * @brief 当前云台状态。
     * 用于判断状态切换
     */
    GimbalState CurrentSate;

    /*-----------------------------状态机-----------------------------*/

    /**
     * @brief 放松状态机
     */
    GimbalStateRelax RelaxState;

    /**
     * @brief 自瞄状态机
     */
    GimbalStateSearch SearchState;

    /**
     * @brief 遥控状态机
     */
    GimbalStateRemoteControl RemoteControlState;

    /**
     * @brief 底盘跟随云台状态机
     */
    GimbalStateFPS FPSState;

    /**
     * @brief 构造函数
     * 默认模式是放松模式。
     * 将电机注册传入不同的状态机。
     */
    GimbalController() : RelaxState(&YawMotor, &PitchMotor),
                         RemoteControlState(&YawMotor, &PitchMotor),
                         SearchState(&YawMotor, &PitchMotor),
                         FPSState(&YawMotor, &PitchMotor)
    {
        this->CurrentSate = STATE_RELAX;
    }

    /**
     * @brief 析构函数。
     */
    ~GimbalController() {};

    /**
     * @brief 初始化函数。
     * 初始化所有的底盘电机，注册电机到MotorHandler。
     * 电机的id在这里设置
     */
    void init();

    /**
     * @brief 运行函数。
     * 根据遥控器的状态，选择不同的状态。
     */
    void run() override;

    /**
     * @brief 改变状态。
     * 用于简化run函数中的状态切换。
     */
    void ChangeState(StateMachine state)
    {
    }

    /**
     * @brief 获取云台控制类的单例。
     * @return GimbalController* 返回云台控制类的单例。
     */
    static GimbalController *instance()
    {
        static GimbalController instance;
        return &instance;
    }
};

#endif

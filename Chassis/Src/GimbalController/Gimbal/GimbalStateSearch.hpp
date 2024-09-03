#ifndef GIMBAL_STATE_SEARCH_HPP
#define GIMBAL_STATE_SEARCH_HPP

#include "StateMachine.hpp"

//#include "bsp_USB.h"

#include "GMMotorHandler.hpp"
#include "Dr16.hpp"
#include "GM6020.hpp"
#include "GM3508.hpp"
#include "GM2006.hpp"
#include "AHRS.hpp"

/**
 * @class GimbalStateSearch
 * @brief 云台搜索状态机，用于哨兵自瞄？
 * @todo 未完成，可能需要等待视觉组的接口。
 */
class GimbalStateSearch : public StateMachine
{
public:
    /**
     * @brief 构造函数。
     */
    GimbalStateSearch(GM6020 *YawMotor,
                      GM6020 *PitchMotor) : YawMotor(YawMotor),
                                            PitchMotor(PitchMotor){};

    /**
     * @brief 析构函数。
     */
    ~GimbalStateSearch(){};

    /**
     * @brief 云台Yaw轴电机。
     */
    GM6020 *YawMotor;

    /**
     * @brief 云台Pitch轴电机。
     */
    GM6020 *PitchMotor;

    /**
     * @brief 视觉发过来云台Yaw轴电机位置。
     * @todo 未完成
     */
    float PitchSet_Ref;
    
    /**
     * @brief 视觉发过来云台Pitch轴电机位置。
     * @todo 未完成
     */
    float YawSet_Ref;

    /**
     * @brief 四元数，用于发送给视觉。
     */
    float Q[4];

    /**
     * @brief 初始化放松状态。
     */
    void init() override;

    /**
     * @brief 进入放松状态。
     */
    void enter() override;

    /**
     * @brief 执行放松状态。
     */
    void execute() override;

    /**
     * @brief 退出放松状态。
     */
    void exit() override;

    /**
     * @brief 运行放松状态。
     */
    void run() override;
};

#endif

#ifndef DR16_HPP
#define DR16_HPP

#include "main.h"
#include "bsp_usart.h"
#include "remoteControl.h"

/*------------------------------------类定义------------------------------------*/

/**
 * @class Dr16
 * @brief 提供遥控器数据处理功能。
 *
 * 该类封装了遥控器操作，包括遥控器按键和摇杆的数据读取与处理。
 */
class Dr16
{
public:
    /**
     * @enum PC_KEY_TYPE
     * @brief 定义电脑按键的类型。
     */
    typedef enum
    {
        PC_KEY_W = ((uint16_t)1 << 0),     ///< 按键W
        PC_KEY_S = ((uint16_t)1 << 1),     ///< 按键S
        PC_KEY_A = ((uint16_t)1 << 2),     ///< 按键A
        PC_KEY_D = ((uint16_t)1 << 3),     ///< 按键D
        PC_KEY_SHIFT = ((uint16_t)1 << 4), ///< 按键Shift
        PC_KEY_CTRL = ((uint16_t)1 << 5),  ///< 按键Ctrl
        PC_KEY_Q = ((uint16_t)1 << 6),     ///< 按键Q
        PC_KEY_E = ((uint16_t)1 << 7),     ///< 按键E
        PC_KEY_R = ((uint16_t)1 << 8),     ///< 按键R
        PC_KEY_F = ((uint16_t)1 << 9),     ///< 按键F
        PC_KEY_G = ((uint16_t)1 << 10),    ///< 按键G
        PC_KEY_Z = ((uint16_t)1 << 11),    ///< 按键Z
        PC_KEY_X = ((uint16_t)1 << 12),    ///< 按键X
        PC_KEY_C = ((uint16_t)1 << 13),    ///< 按键C
        PC_KEY_V = ((uint16_t)1 << 14),    ///< 按键V
        PC_KEY_B = ((uint16_t)1 << 15),    ///< 按键B
    } PC_KEY_TYPE;

    /**
     * @enum PC_KEY_STATE_TYPE
     * @brief 定义电脑按键的状态类型。
     */
    typedef enum
    {
        PC_KEY_DOWN, ///< 按键保持按下
        PC_KEY_UP,   ///< 按键保持松开
        PC_KEY_FALL, ///< 按键下降沿
        PC_KEY_RISE  ///< 按键上升沿
    } PC_KEY_STATE_TYPE;

    /*-------------------------遥控器遥感拨杆相关-------------------------*/

    /**
     * @enum RC_SWITCH_STATE_TYPE
     * @brief 定义遥控器开关的状态变化类型。
     * @note 用于判断遥控器开关的状态。
     */
    typedef enum
    {
        RC_SW_UP = 1,   ///< 开关上
        RC_SW_MID = 3,  ///< 开关中
        RC_SW_DOWN = 2, ///< 开关下

        RC_SWITCH_M2D = 4, ///< 从中到下
        RC_SWITCH_M2U = 5, ///< 从中到上
        RC_SWITCH_D2M = 6, ///< 从下到中
        RC_SWITCH_U2M = 7  ///< 从上到中
    } RC_SWITCH_STATE_TYPE;

    /**
     * @struct  KeyStatus
     * @brief 获取电脑按键的状态。
     * @param current_state 当前按键状态
     * @param previous_state 上一次按键状态
     */
    typedef struct
    {
        uint16_t current_state;
        uint16_t previous_state;
    } KeyStatus;
    /**
     * @brief 构造函数，初始化遥控器接口。
     */
    Dr16();

    /**
     * @brief 析构函数。
     */
    ~Dr16();

    RC_ctrl_t *rc_raw; ///< 指向遥控器原始数据的指针。

    /*--------------------------------------摇杆--------------------------------------*/
    float rc_right_x; ///< 遥控器右摇杆X轴数据，范围为[-1, 1]。
    float rc_right_y; ///< 遥控器右摇杆Y轴数据，范围为[-1, 1]。
    float rc_left_x;  ///< 遥控器左摇杆X轴数据，范围为[-1, 1]。
    float rc_left_y;  ///< 遥控器左摇杆Y轴数据，范围为[-1, 1]。

    /*--------------------------------------拨杆--------------------------------------*/
    RC_SWITCH_STATE_TYPE left_sw;        ///< 遥控器左侧SW1开关当前状态。
    RC_SWITCH_STATE_TYPE left_prev_sw;   ///< 遥控器左侧SW1开关上一次的状态。
    RC_SWITCH_STATE_TYPE left_sw_change; ///< 遥控器左侧SW1开关状态的变化。

    RC_SWITCH_STATE_TYPE right_sw;        ///< 遥控器右侧SW2开关当前状态。
    RC_SWITCH_STATE_TYPE right_prev_sw;   ///< 遥控器右侧SW2开关上一次的状态。
    RC_SWITCH_STATE_TYPE right_sw_change; ///< 遥控器右侧SW2开关状态的变化。

    /**
     * @brief 将遥控器摇杆的通道数据映射到[-1, 1]的范围。
     * @param ch 遥控器摇杆通道的原始数据。
     * @return 映射后的数据，范围为[-1, 1]。
     */
    float mapAvix(int16_t ch);

    /**
     * @brief 更新遥控器状态。
     */
    void updateRcStatus();

    /*---------------------------------------------PC按键相关---------------------------------------------*/

    KeyStatus key_status = {0, 0};

    /**
     * @brief 更新按键状态。
     */
    void updateKeyStatus();

    /**
     * @brief 获取按键状态。
     * @param key 按键类型。
     * @return 按键状态。
     */
    PC_KEY_STATE_TYPE getKeyStatus(PC_KEY_TYPE key);

    /*-----------------------------------------主要函数-----------------------------------------*/

    /**
     * @brief 更新遥控器数据。遥控器数据和键盘数据的更新。
     * @note 该函数现在在主循环中调用。
     * @todo 考虑将这个函数放在中断中调用，实现当有数据到达时才更新。
     * @todo 键盘按键的Key_Fall还没进行测试。
     */
    void updateData();

    static Dr16 *instance() ///< 遥控器类的实例。
    {
        static Dr16 instance;
        return &instance;
    }
};

#endif // DR16_HPP

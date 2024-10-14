#ifndef REMOTECONTROL_H
#define REMOTECONTROL_H

#include "main.h"

#ifdef __cplusplus
extern "C"
{
#endif
#define SBUS_RX_BUF_NUM 36u ///< SBUS接收缓冲区大小

#define RC_FRAME_LENGTH 18u ///< 遥控器数据帧长度

#define NULL 0 ///< 空指针

/**
 * @def RC_CH_VALUE_MIN
 * @brief 遥控器通道值的最小值
 */
#define RC_CH_VALUE_MIN ((uint16_t)364)

/**
 * @def RC_CH_VALUE_OFFSET
 * @brief 遥控器通道值的中间值
 */
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)

/**
 * @def RC_CH_VALUE_MAX
 * @brief 遥控器通道值的最大值
 */
#define RC_CH_VALUE_MAX ((uint16_t)1684)

/**
 * @def RC_CH_OFFSET_MAX
 * @brief 遥控器通道最大偏移值
 */
#define RC_CH_OFFSET_MAX ((uint16_t)660)

// /* ----------------------- PC Key Definition-------------------------------- */
// #define KEY_PRESSED_OFFSET_W ((uint16_t)1 << 0)
// #define KEY_PRESSED_OFFSET_S ((uint16_t)1 << 1)
// #define KEY_PRESSED_OFFSET_A ((uint16_t)1 << 2)
// #define KEY_PRESSED_OFFSET_D ((uint16_t)1 << 3)
// #define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)1 << 4)
// #define KEY_PRESSED_OFFSET_CTRL ((uint16_t)1 << 5)
// #define KEY_PRESSED_OFFSET_Q ((uint16_t)1 << 6)
// #define KEY_PRESSED_OFFSET_E ((uint16_t)1 << 7)
// #define KEY_PRESSED_OFFSET_R ((uint16_t)1 << 8)
// #define KEY_PRESSED_OFFSET_F ((uint16_t)1 << 9)
// #define KEY_PRESSED_OFFSET_G ((uint16_t)1 << 10)
// #define KEY_PRESSED_OFFSET_Z ((uint16_t)1 << 11)
// #define KEY_PRESSED_OFFSET_X ((uint16_t)1 << 12)
// #define KEY_PRESSED_OFFSET_C ((uint16_t)1 << 13)
// #define KEY_PRESSED_OFFSET_V ((uint16_t)1 << 14)
// #define KEY_PRESSED_OFFSET_B ((uint16_t)1 << 15)
/* ----------------------- Data Struct ------------------------------------- */

/**
 * @brief 遥控器数据结构。
 *
 * 此结构用于存储遥控器的完整状态信息，包括操纵杆数据、按钮状态和附加开关。
 */

#pragma pack(push) //保存对齐状态
#pragma pack(1)
typedef  struct
{
    /**
     * @brief 遥控器的通道和开关。
     *
     * 此子结构用于存储来自遥控器的模拟通道和数字开关输入。
     */
    struct
    {
        int16_t ch[5]; /**< 遥控器的通道值，通常用于模拟操纵杆输入。 */
        char s[2];     /**< 开关状态，通常用于二进制开关输入。 */
    } rc;

    /**
     * @brief 鼠标数据结构。
     *
     * 此子结构存储与鼠标相关的数据，这些数据可能用于控制机器人或与计算机模拟接口。
     */
     struct
    {
        int16_t x;       /**< 鼠标 X 轴移动。 */
        int16_t y;       /**< 鼠标 Y 轴移动。 */
        int16_t z;       /**< 鼠标滚轮移动。 */
        uint8_t press_l; /**< 左鼠标按钮按压。 */
        uint8_t press_r; /**< 右鼠标按钮按压。 */
    } mouse;

    /**
     * @brief 键盘按键值。
     *
     * 此子结构用于存储遥控器键盘接口上特定按键的状态。
     */
     struct
    {
        uint16_t v; /**< 遥控器键盘接口上按下的键值。 */
    } key;

} RC_ctrl_t;
#pragma pack(pop) //恢复对齐状态
/**
 * @brief 初始化Dr16遥控器相关配置
 * 调用usat3_init()初始化串口3
 */
void remote_control_init(void);

/**
 * @brief 获取遥控器原始数据指针，在Dr16类中调用
 * @return RC_ctrl_t* 遥控器原始数据指针
 */
RC_ctrl_t *get_remote_control_raw(void);

/**
 * @brief 解析遥控器原始数据
 * @param sbus_buf 遥控器原始数据
 * @param rc_raw 遥控器原始数据结构体
 */
static void SBUS_TO_RC(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_raw);

/**
 * @brief 处理遥控器数据
 * @note 用于处理遥控器数据，在USART3_IRQHandler中调用
 */
void process_remote_control_data(void);

#ifdef __cplusplus
}
#endif
#endif // REMOTECONTROL_H

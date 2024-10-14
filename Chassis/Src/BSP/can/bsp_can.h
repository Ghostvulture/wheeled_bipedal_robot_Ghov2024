#ifndef BSP_CAN_H
#define BSP_CAN_H


#ifdef __cplusplus
extern "C"
{
#endif
	
#include "main.h"
/**
 * @brief 初始化CAN滤波器配置。
 * 设置CAN硬件的滤波器，用于优化接收数据的处理。
 * 更多信息，请参考原文，链接：https://blog.csdn.net/weixin_54448108/article/details/128570593
 */
void can_filter_init(void);

/**
 * @brief 发送CAN数据。
 * @param hcan 指向CAN句柄的指针，用于配置CAN传输。
 * @param StdId CAN消息的标准标识符
 * @param msg 发送的数据
 * @param len 数据长度
 * @note 该函数用于发送CAN数据，目前只支持标准帧
 */
void can_sendData(CAN_HandleTypeDef *hcan, uint32_t Id, uint8_t *msg, uint16_t len);

/**
 * @brief 从CAN接收数据。
 * @param rx_header 指向接收到的数据的指针。
 * @param receiveData 指向接收数据的指针，用于存储接收的数据。
 */
void can_recieveData();

#ifdef __cplusplus
}
#endif
#endif // BSP_CAN_HPP

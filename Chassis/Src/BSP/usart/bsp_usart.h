#ifndef BSP_USART_H
#define BSP_USART_H

#include "main.h"
#ifdef __cplusplus
extern "C"
{
#endif
/**
 * @brief 初始化串口。
*/
void usart_init(void);

/**
 * @brief 初始化串口1。
*/
void usart1_init(void);

/**
 * @brief 初始化串口3。
 * @param rx1_buf DMA接收缓冲区1
 * @param rx2_buf DMA接收缓冲区2
 * @param dma_buf_num DMA缓冲区大小
 * @note 串口3用于接收遥控器数据。需要在初始化时传入DMA接收缓冲区1、DMA接收缓冲区2和DMA缓冲区大小，因此只能在remoteControl.c中调用。
*/
void usart3_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

#ifdef __cplusplus
}
#endif
#endif //  __BSP_USART_H

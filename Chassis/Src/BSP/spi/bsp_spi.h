#ifndef BSP_SPI_H
#define BSP_SPI_H

#include "main.h"
#ifdef __cplusplus
extern "C"
{
#endif
typedef enum
{
    SPI_BLOCK_MODE = 0,
    SPI_IT_MODE,
    SPI_DMA_MODE
} SPI_WORK_MODE;

/**
 * @brief  SPI 初始化
 * 只是形式上封装
 */
void spi_init(void);

/**
 * @brief  SPI 发送数据
 * @param  SPIx: SPI外设
 * @param  data: 数据
 * @param  len: 数据长度
 * @todo 加入阻塞、中断、DMA模式，加入超时机制以及错误处理
 */
void spi_sendData(SPI_HandleTypeDef *hspi, uint8_t *data, uint8_t len, SPI_WORK_MODE mode);

/**
 * @brief  SPI 读取数据
 * @param  SPIx: SPI外设
 * @param  addr: 寄存器地址
 * @param  data: 数据
 * @param  len: 数据长度
 * @todo 加入阻塞、中断、DMA模式，加入超时机制以及错误处理
 */
void spi_readData(SPI_HandleTypeDef *hspi, uint8_t *pData, uint8_t len, SPI_WORK_MODE mode);

#ifdef __cplusplus
}
#endif
#endif // !BSP_SPI_Hs

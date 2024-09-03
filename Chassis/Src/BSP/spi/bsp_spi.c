#include "bsp_spi.h"

void spi_init()
{
}

void spi_sendData(SPI_HandleTypeDef *hspi, uint8_t *data, uint8_t len, SPI_WORK_MODE mode)
{
    switch (mode)
    {
    case SPI_BLOCK_MODE: // 阻塞模式
        uint8_t pTxData;
        for (int i = 0; i < len; i++)
        {
            pTxData = data[i];
            HAL_SPI_Transmit(hspi, &pTxData, 1, 1000);
            while (HAL_SPI_GetState(hspi) == HAL_SPI_STATE_BUSY_TX) // 等待发送完成，这样的设计可能不需要，也可能锁死，需要测试
                ;
        }
        break;

    case SPI_IT_MODE: // 中断模式
        HAL_SPI_Transmit_IT(hspi, data, len);
        break;

    case SPI_DMA_MODE: // DMA模式
        HAL_SPI_Transmit_DMA(hspi, data, len);
        break;

    default: // 未知的模式
        break;
    }
}

void spi_readData(SPI_HandleTypeDef *hspi, uint8_t *pData, uint8_t len, SPI_WORK_MODE mode)
{
    switch (mode)
    {
    case SPI_BLOCK_MODE: // 阻塞模式
        uint8_t pRxData;

        for (int i = 0; i < len; i++)
        {
            HAL_SPI_Receive(hspi, &pRxData, 1, 1000);               // 1000ms超时
            while (HAL_SPI_GetState(hspi) == HAL_SPI_STATE_BUSY_RX) // 等待接收完成，这样的设计可能不需要，也可能锁死，需要测试
                ;
            pData[i] = pRxData;
        }
        break;

    case SPI_IT_MODE: // 中断模式
        HAL_SPI_Receive_IT(hspi, pData, len);
        break;

    case SPI_DMA_MODE: // DMA模式
        HAL_SPI_Receive_DMA(hspi, pData, len);
        break;

    default: // 未知的模式
        break;
    }
}

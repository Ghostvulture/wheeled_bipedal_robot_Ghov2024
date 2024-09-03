#include "bsp_usart.h"
#include "remoteControl.h"
#include "stm32f4xx_it.h"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

void usart_init(void)
{
  usart1_init();
}

void usart1_init() {};

void usart3_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
  // 使能DMA窗口接收
  SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);
  // 使能空闲中断
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
  // 失效DMA
  __HAL_DMA_DISABLE(&hdma_usart3_rx);
  while (hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
  {
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
  }

  hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR);

  // 内存缓冲区1
  hdma_usart3_rx.Instance->M0AR = (uint32_t)(rx1_buf);
  // 内存缓冲区2
  hdma_usart3_rx.Instance->M1AR = (uint32_t)(rx2_buf);
  // 数据长度
  hdma_usart3_rx.Instance->NDTR = dma_buf_num;

  // 使能双缓冲区
  SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);
  // 使能DMA
  __HAL_DMA_ENABLE(&hdma_usart3_rx);

  /*---------------------LL库配置----------------------*/
  // LL_USART_EnableDMAReq_RX(USART3); // 使能串口3的DMA接收请求

  // LL_USART_ClearFlag_IDLE(USART3); // 清除空闲中断标志位
  // LL_USART_EnableIT_IDLE(USART3);	 // 使能空闲中断
  // /* -------------- Configure DMA -----------------------------------------*/
  // LL_DMA_InitTypeDef DMA_InitStructure;
  // LL_DMA_DeInit(DMA1, LL_DMA_STREAM_1);

  // DMA_InitStructure.Channel = LL_DMA_CHANNEL_4;
  // DMA_InitStructure.PeriphOrM2MSrcAddress = (uint32_t) & (USART3->DR);
  // DMA_InitStructure.MemoryOrM2MDstAddress = (uint32_t)rx1_buf;
  // DMA_InitStructure.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
  // DMA_InitStructure.NbData = dma_buf_num;
  // DMA_InitStructure.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
  // DMA_InitStructure.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
  // DMA_InitStructure.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
  // DMA_InitStructure.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
  // DMA_InitStructure.Mode = LL_DMA_MODE_CIRCULAR;
  // DMA_InitStructure.Priority = LL_DMA_PRIORITY_VERYHIGH;
  // DMA_InitStructure.FIFOMode = LL_DMA_FIFOMODE_DISABLE;
  // DMA_InitStructure.FIFOThreshold = LL_DMA_FIFOTHRESHOLD_1_4;
  // DMA_InitStructure.MemBurst = LL_DMA_MBURST_SINGLE;
  // DMA_InitStructure.PeriphBurst = LL_DMA_PBURST_SINGLE;
  // LL_DMA_Init(DMA1, LL_DMA_STREAM_1, &DMA_InitStructure);				// 初始化DMA，尽管MX有相关初始化，但是有些设置需要重新设置
  // LL_DMA_SetMemory1Address(DMA1, LL_DMA_STREAM_1, (uint32_t)rx2_buf); // 设置DMA的第一个缓冲区地址
  // LL_DMA_EnableDoubleBufferMode(DMA1, LL_DMA_STREAM_1);				// 使能DMA的双缓冲模式
  // LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_1);
  // LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1); // 使能DMA
}

void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */
  process_remote_control_data();
  /* USER CODE END USART3_IRQn 1 */
}

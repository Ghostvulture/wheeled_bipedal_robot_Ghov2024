#include "remoteControl.h"
#include "bsp_usart.h"
#include "main.h"
// #include "Dr16.hpp"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

/**
 * @brief 遥控器原始数据结构体。
 */
RC_ctrl_t rc_raw;

// 遥控器DMA接收缓冲区
static uint8_t SBUS_rx_buf[2][SBUS_RX_BUF_NUM];

void remote_control_init(void)
{
    usart3_init(SBUS_rx_buf[0], SBUS_rx_buf[1], SBUS_RX_BUF_NUM);
}

RC_ctrl_t *get_remote_control_raw()
{
    return &rc_raw;
}

static void SBUS_TO_RC(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_raw)
{
    if (sbus_buf == NULL || rc_raw == NULL)
    {
        return;
    }

    rc_raw->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_raw->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_raw->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                        (sbus_buf[4] << 10)) &
                       0x07ff;
    rc_raw->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_raw->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                       //!< Switch left
    rc_raw->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                  //!< Switch right

    rc_raw->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);   //!< Mouse X axis
    rc_raw->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);   //!< Mouse Y axis
    rc_raw->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8); //!< Mouse Z axis

    rc_raw->mouse.press_l = sbus_buf[12]; //!< Mouse Left Is Press ?
    rc_raw->mouse.press_r = sbus_buf[13]; //!< Mouse Right Is Press ?

    rc_raw->key.v = sbus_buf[14] | (sbus_buf[15] << 8); //!< KeyBoard value

    rc_raw->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8); // NULL

    rc_raw->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_raw->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_raw->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_raw->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_raw->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}

void process_remote_control_data()
{
    if (huart3.Instance->SR & UART_FLAG_RXNE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if (USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            // disable DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            // get receive data length, length = set_data_length - remain_length
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            // reset set_data_lenght
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            // set memory buffer 1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;

            // enable DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if (this_time_rx_len == RC_FRAME_LENGTH)
            {
                SBUS_TO_RC(SBUS_rx_buf[0], &rc_raw);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            // disable DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            // get receive data length, length = set_data_length - remain_length
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            // reset set_data_lenght
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            // set memory buffer 0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

            // enable DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if (this_time_rx_len == RC_FRAME_LENGTH)
            {
                SBUS_TO_RC(SBUS_rx_buf[1], &rc_raw);
            }
        }
    }

    /*--------------------------------------LL库下的数据处理--------------------------------------*/
 
    // if (LL_USART_IsActiveFlag_RXNE(USART3) != RESET)
    // {
    //     LL_USART_ReceiveData9(USART3);
    // }
    // else if (LL_USART_IsActiveFlag_IDLE(USART3) != RESET)
    // {
    //     static uint16_t this_time_rx_len = 0;
    //     LL_USART_ReceiveData9(USART3);

    //     if (LL_DMA_GetCurrentTargetMem(DMA1, LL_DMA_STREAM_1) == LL_DMA_CURRENTTARGETMEM0)
    //     {
    //         LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_1);
    //         this_time_rx_len = SBUS_RX_BUF_NUM - LL_DMA_GetDataLength(DMA1, LL_DMA_STREAM_1);
    //         LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_1, SBUS_RX_BUF_NUM);
    //         LL_DMA_SetCurrentTargetMem(DMA1, LL_DMA_STREAM_1, LL_DMA_CURRENTTARGETMEM1);

    //         LL_DMA_ClearFlag_TC1(DMA1);
    //         LL_DMA_ClearFlag_HT1(DMA1);
    //         LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);
    //         if (this_time_rx_len == RC_FRAME_LENGTH)
    //         {
    //             SBUS_TO_RC(SBUS_rx_buf[0], &rc_raw);
    //         }
    //     }
    //     else
    //     {
    //         LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_1);
    //         this_time_rx_len = SBUS_RX_BUF_NUM - LL_DMA_GetDataLength(DMA1, LL_DMA_STREAM_1);
    //         LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_1, SBUS_RX_BUF_NUM);
    //         LL_DMA_SetCurrentTargetMem(DMA1, LL_DMA_STREAM_1, LL_DMA_CURRENTTARGETMEM0);

    //         LL_DMA_ClearFlag_TC1(DMA1);
    //         LL_DMA_ClearFlag_HT1(DMA1);
    //         LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);

    //         if (this_time_rx_len == RC_FRAME_LENGTH)
    //         {
    //             SBUS_TO_RC(SBUS_rx_buf[1], &rc_raw);
    //         }
    //     }
    // }
}

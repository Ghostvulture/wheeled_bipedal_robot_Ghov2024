#include "bsp_can.h"
#include "main.h"

#include "BoardConnectivity.hpp"
#include "GMMotorhandler.hpp"
#include "LKMotorHandler.hpp"
#include "cpp_solution.hpp"


/*
对于C板，有两个can。
*/
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/**
 * @brief 初始化CAN滤波器配置。
 * 设置CAN硬件的滤波器，用于优化接收数据的处理。
 * 更多信息，请参考原文，链接：https://blog.csdn.net/weixin_54448108/article/details/128570593
 */
void can_filter_init(void)
{
    CAN_FilterTypeDef can_filter_st;                   ///< 定义过滤器结构体
    can_filter_st.FilterActivation = ENABLE;           ///< ENABLE使能过滤器
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;  ///< 设置过滤器模式--标识符屏蔽位模式
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT; ///< 过滤器的位宽 32 位
    can_filter_st.FilterIdHigh = 0x0000;               ///< ID高位
    can_filter_st.FilterIdLow = 0x0000;                ///< ID低位
    can_filter_st.FilterMaskIdHigh = 0x0000;           ///< 过滤器掩码高位
    can_filter_st.FilterMaskIdLow = 0x0000;            ///< 过滤器掩码低位

    can_filter_st.FilterBank = 0;                                      ///< 过滤器组-双CAN可指定0~27
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;                 ///< 与过滤器组管理的 FIFO
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);                      ///< HAL库配置过滤器函数
    HAL_CAN_Start(&hcan1);                                             ///< 使能CAN1控制器
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); ///< 使能CAN的各种中断

    can_filter_st.SlaveStartFilterBank = 14;                           ///< 双CAN模式下规定CAN的主从模式的过滤器分配，从过滤器为14
    can_filter_st.FilterBank = 14;                                     ///< 过滤器组-双CAN可指定0~27
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);                      ///< HAL库配置过滤器函数
    HAL_CAN_Start(&hcan2);                                             ///< 使能CAN2控制器
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING); ///< 使能CAN的各种中断
}

void can_sendData(CAN_HandleTypeDef *hcan, uint32_t Id, uint8_t *msg, uint16_t len)
{
    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0)
    {
        // 如果邮箱已满，可以选择等待或者采取其他操作
        // 例如，可以添加一个超时机制或者直接返回错误状态
        return;
    }
    CAN_TxHeaderTypeDef tx_header; ///< 定义发送数据结构体
    uint32_t send_mail_box;        ///< 定义发送邮箱
    tx_header.StdId = Id;          ///< 标准ID
    tx_header.IDE = CAN_ID_STD;    ///< 标准帧
    tx_header.RTR = CAN_RTR_DATA;  ///< 远程传输请求
    tx_header.DLC = len;           ///< 数据长度

    HAL_CAN_AddTxMessage(hcan, &tx_header, msg, &send_mail_box);
}

/**
 * @todo 待完成整个函数
 * @brief 更新从CAN接收的电机数据。
 * @param index 电机的索引，用于识别特定的电机。
 * @param rx_header 指向接收到的数据的指针。
 * @param can_receive_data 指向电机测量数据结构的指针，用于存储更新的数据。
 */
void can_recieveData()
{
}
extern void can_callback_cpp(CAN_HandleTypeDef *hcan);
/**
 * @brief CAN接收中断回调函数，所有反馈在can上的数据会在这里根据ID进行分类并处理。
 * @param hcan CAN句柄
 * @todo 完成对不同电机的数据处理
 * @todo 完成对大疆电机的处理
 * @todo 完成对其他电机的处理
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{


    // /*-------------------------------------------------大疆电机数据-------------------------------------------------*/
    // if (rx_header.StdId >= 0x201 && rx_header.StdId <= 0x208)
    // {
    //     if (hcan->Instance == CAN1)
    //     {
    //         GMMotorHandler::instance()->processRawData(hcan, rx_data, int(rx_header.StdId - 0x201));
    //     }
    //     else if (hcan->Instance == CAN2) // 处理CAN2的数据
    //     {
    //         GMMotorHandler::instance()->processRawData(hcan, rx_data, int(rx_header.StdId - 0x201));
    //     }
    // }
    // /*--------------------------------------------------LK电机数据--------------------------------------------------*/
    // else if (rx_header.StdId >= 0x140 && rx_header.StdId <= 0x160)
    // {
    //         if (hcan->Instance == CAN1)
    //         {
    //                 LKMotorHandler::instance()->processRawData(hcan, rx_data, int(rx_header.StdId - 0x141));
    //         }
    //         else if (hcan->Instance == CAN2) // 处理CAN2的数据
    //         {
    //                 LKMotorHandler::instance()->processRawData(hcan, rx_data, int(rx_header.StdId - 0x141));
    //         }
    // }

    can_callback_cpp(hcan);

    /*--------------------------------------------------LK电机数据--------------------------------------------------*/
    // else if (rx_header.StdId >= 0xB1 && rx_header.StdId <= 0xB8) // 板件通讯信息处理
    // {
    //     if (hcan->Instance == CAN1)
    //     {
    //         BoardConnectivity::instance()->BoardConnectivity_Add2Memory(rx_data, rx_header.StdId, rx_header.DLC, BoardConnectivity::BOARD_CONNECTIVITY_CAN_1, BoardConnectivity::BOARD_CONNECTIVITY_RECEIVE);
    //     }
    //     else if (hcan->Instance == CAN2)
    //     {
    //         BoardConnectivity::instance()->BoardConnectivity_Add2Memory(rx_data, rx_header.StdId, rx_header.DLC, BoardConnectivity::BOARD_CONNECTIVITY_CAN_2, BoardConnectivity::BOARD_CONNECTIVITY_RECEIVE);
    //     }
    // }

}


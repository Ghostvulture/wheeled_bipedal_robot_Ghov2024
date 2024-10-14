#include "cpp_solution.hpp"

#include "AHRS.hpp"
#include "GMMotorhandler.hpp"
#include "LKMotorHandler.hpp"
#include "bsp_can.h"
#include "IST8310.hpp"



void can_callback_cpp(CAN_HandleTypeDef *hcan)
{

    CAN_RxHeaderTypeDef rx_header;
    // 接收数据
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    /*-------------------------------------------------大疆电机数据-------------------------------------------------*/
    if (rx_header.StdId >= 0x201 && rx_header.StdId <= 0x208)
    {
        if (hcan->Instance == CAN1)
        {
            GMMotorHandler::instance()->processRawData(hcan, rx_data, int(rx_header.StdId - 0x201));
        }
        else if (hcan->Instance == CAN2) // 处理CAN2的数据
        {
            GMMotorHandler::instance()->processRawData(hcan, rx_data, int(rx_header.StdId - 0x201));
        }
    }
    /*--------------------------------------------------LK电机数据--------------------------------------------------*/
    else if (rx_header.StdId >= 0x140 && rx_header.StdId <= 0x160)
    {
            if (hcan->Instance == CAN1)
            {
                    LKMotorHandler::instance()->processRawData(hcan, rx_data, int(rx_header.StdId - 0x141));
            }
            else if (hcan->Instance == CAN2) // 处理CAN2的数据
            {
                    LKMotorHandler::instance()->processRawData(hcan, rx_data, int(rx_header.StdId - 0x141));
            }
    }
    else // 未知的ID，需要进行错误处理
    {
    }
}

void ahrs_update()
{
    AHRS::instance()->AHRS_Update();
}

void ist8310_getMagData()
{
    IST8310::instance()->getMagData();
}
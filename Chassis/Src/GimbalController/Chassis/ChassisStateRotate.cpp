#include "ChassisStateRotate.hpp"

void ChassisStateRotate::init()
{
    // 初始化速度为0
    Vx = 0.0f;
    Vy = 0.0f;
    // Vw = 0.0f;
    Vw = 4.0f;

    uint8_t xyData[8];
    uint8_t wData[8];
    BoardConnectivity::instance()->BoardConnectivity_Float2Byte(Vx, xyData);
    BoardConnectivity::instance()->BoardConnectivity_Float2Byte(Vy, xyData + 4);
    BoardConnectivity::instance()->BoardConnectivity_Float2Byte(Vw, wData);
    BoardConnectivity::instance()->BoardConnectivity_Float2Byte(Vw, wData + 4);

    BoardConnectivity::instance()->BoardConnectivity_Add2Memory(xyData, 0xB1, 8, BoardConnectivity::BOARD_CONNECTIVITY_CAN_2, BoardConnectivity::BOARD_CONNECTIVITY_SEND);
    BoardConnectivity::instance()->BoardConnectivity_Add2Memory(wData, 0xB2, 8, BoardConnectivity::BOARD_CONNECTIVITY_CAN_2, BoardConnectivity::BOARD_CONNECTIVITY_SEND);
}

void ChassisStateRotate::enter()
{
    // 将遥控器的数据转化为底盘速度，在这里乘以一个常数，是为了调整速度的大小。单位应该为m/s。
    Vx = Dr16::instance()->rc_left_y * 5;
    Vy = Dr16::instance()->rc_left_x * 5;
    Vw = 4.50f; // 单位为rad/s，因此不需要乘以常数，或者乘以一个较小的常数。不要超过Pi，因为在PID算法里面，总是计算最短距离。
}

void ChassisStateRotate::execute()
{
    // 发送数据给底盘
    uint8_t xyData[8];
    uint8_t wData[8];
    BoardConnectivity::instance()->BoardConnectivity_Float2Byte(Vx, xyData);
    BoardConnectivity::instance()->BoardConnectivity_Float2Byte(Vy, xyData + 4);
    BoardConnectivity::instance()->BoardConnectivity_Float2Byte(Vw, wData);
    BoardConnectivity::instance()->BoardConnectivity_Float2Byte(Vw, wData + 4);

    BoardConnectivity::instance()->BoardConnectivity_Add2Memory(xyData, 0xB1, 8, BoardConnectivity::BOARD_CONNECTIVITY_CAN_2, BoardConnectivity::BOARD_CONNECTIVITY_SEND);
    BoardConnectivity::instance()->BoardConnectivity_Add2Memory(wData, 0xB2, 8, BoardConnectivity::BOARD_CONNECTIVITY_CAN_2, BoardConnectivity::BOARD_CONNECTIVITY_SEND);
}

void ChassisStateRotate::exit()
{
}

void ChassisStateRotate::run()
{
    enter();
    execute();
    exit();
}

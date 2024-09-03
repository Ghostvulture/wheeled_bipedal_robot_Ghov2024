#include "ChassisStateRemoteControl.hpp"

void ChassisStateRemoteControl::init()
{   
    // 初始化速度为0
    Vx = 0.0f;
    Vy = 0.0f;
    Vw = 0.0f;

    // 设置滤波器参数
    VxFilter.SetTau(0.16f);
    VxFilter.SetUpdatePeriod(5);

    VyFilter.SetTau(0.12f);
    VyFilter.SetUpdatePeriod(5);

    VwFilter.SetTau(0.04f);
    VwFilter.SetUpdatePeriod(5);

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

void ChassisStateRemoteControl::enter()
{
    // 将遥控器的数据转化为底盘速度，在这里乘以一个常数，是为了调整速度的大小。单位应该为m/s。
    Vx = Dr16::instance()->rc_left_y * 5;
    Vy = Dr16::instance()->rc_left_x * 5;
    Vw = 0; // 单位为rad/s

    // 对速度进行滤波
    VxFilter.SetInput(Vx);
    VxFilter.Update();
    Vx = VxFilter.GetResult();

    // 对速度进行滤波
    VyFilter.SetInput(Vy);
    VyFilter.Update();
    Vy = VyFilter.GetResult();
}

void ChassisStateRemoteControl::execute()
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

void ChassisStateRemoteControl::exit()
{
}

void ChassisStateRemoteControl::run()
{
    enter();
    execute();
    exit();
}

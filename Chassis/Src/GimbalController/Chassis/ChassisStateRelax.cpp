#include "ChassisStateRelax.hpp"

void ChassisStateRelax::init()
{
    Vx = 0.0f;
    Vy = 0.0f;
    Vw = 0.0f;

    uint8_t xyData[8];
    uint8_t wData[8];
    BoardConnectivity::instance()->BoardConnectivity_Float2Byte(Vx, xyData);
    BoardConnectivity::instance()->BoardConnectivity_Float2Byte(Vy, xyData + 4);
    BoardConnectivity::instance()->BoardConnectivity_Float2Byte(Vw, wData);
    BoardConnectivity::instance()->BoardConnectivity_Float2Byte(Vw, wData + 4);

    BoardConnectivity::instance()->BoardConnectivity_Add2Memory(xyData, 0xB1, 8, BoardConnectivity::BOARD_CONNECTIVITY_CAN_2, BoardConnectivity::BOARD_CONNECTIVITY_SEND);
    BoardConnectivity::instance()->BoardConnectivity_Add2Memory(wData, 0xB2, 8, BoardConnectivity::BOARD_CONNECTIVITY_CAN_2, BoardConnectivity::BOARD_CONNECTIVITY_SEND);
}

void ChassisStateRelax::enter()
{
    Vx = 0.0f;
    Vy = 0.0f;
    Vw = 0.0f;
}

void ChassisStateRelax::execute()
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

void ChassisStateRelax::exit()
{
}

void ChassisStateRelax::run()
{
    enter();
    execute();
    exit();
}

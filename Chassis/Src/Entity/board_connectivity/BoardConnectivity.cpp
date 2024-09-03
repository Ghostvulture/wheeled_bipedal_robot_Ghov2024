#include "BoardConnectivity.hpp"

// 对于C板，有两个can口，分别是hcan1和hcan2，如果使用了其他的板子，需要修改这里
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

BoardConnectivity::BoardConnectivity()
{
    init();
}

BoardConnectivity::~BoardConnectivity()
{
    for (int i = 0; i < 8; i++)
    {
        if (BoardMemory_receive[i].data != nullptr)
        {
            free(BoardMemory_receive[i].data);
        }
        if (BoardMemory_send[i].data != nullptr)
        {
            free(BoardMemory_send[i].data);
        }
    }
}

void BoardConnectivity::init()
{
    for (int i = 0; i < 8; i++)
    {
        BoardMemory_receive[i].data = nullptr;
        BoardMemory_receive[i].len = 0;

        BoardMemory_send[i].data = nullptr;
        BoardMemory_send[i].len = 0;
    }
}

void BoardConnectivity::BoardConnectivity_Add2Memory(uint8_t *data, uint16_t id, uint16_t len, BoardConnectivityType type, BoardConnectivityDirection direction)
{
    if (id > 0xB8 || id < 0xB1)
    {
        return; // id不在范围内
    }
    id -= 0xB1; // id转换为数组索引

    if (direction == BOARD_CONNECTIVITY_SEND)
    {
        // 如果之前有数据，先释放内存
        if (BoardMemory_send[id].data != nullptr)
        {
            free(BoardMemory_send[id].data);
        }

        BoardMemory_send[id].data = (uint8_t *)malloc(len); // 分配内存
        memcpy(BoardMemory_send[id].data, data, len);       // 复制数据
        BoardMemory_send[id].type = type;
        BoardMemory_send[id].id = id + 0xB1;
        BoardMemory_send[id].len = len;
        return;
    }
    else if (direction == BOARD_CONNECTIVITY_RECEIVE)
    {
        // 如果之前有数据，先释放内存
        if (BoardMemory_receive[id].data != nullptr)
        {
            free(BoardMemory_receive[id].data);
        }

        BoardMemory_receive[id].data = (uint8_t *)malloc(len); // 分配内存
        memcpy(BoardMemory_receive[id].data, data, len);       // 复制数据
        BoardMemory_receive[id].type = type;
        BoardMemory_receive[id].id = id + 0xB1;
        BoardMemory_receive[id].len = len;
        return;
    }
}

void BoardConnectivity::BoardConnectivity_Send()
{
    for (int i = 0; i < 8; i++)
    {
        if (BoardMemory_send[i].data != nullptr && BoardMemory_send[i].len != 0)
        {
            switch (BoardMemory_send[i].type)
            {
            case BOARD_CONNECTIVITY_CAN_1:
                can_sendData(&hcan1, BoardMemory_send[i].id, BoardMemory_send[i].data, BoardMemory_send[i].len);
                break;
            case BOARD_CONNECTIVITY_CAN_2:
                can_sendData(&hcan2, BoardMemory_send[i].id, BoardMemory_send[i].data, BoardMemory_send[i].len);
                break;
            case BOARD_CONNECTIVITY_USART:
                break;
            default:
                break;
            }
        }
    }
}

void BoardConnectivity::BoardConnectivity_Float2Byte(float f, uint8_t *byte)
{
    memcpy(byte, &f, sizeof(f));
}

void BoardConnectivity::BoardConnectivity_Byte2Float(uint8_t *byte, float *f)
{
    memcpy(f, byte, sizeof(f));
}

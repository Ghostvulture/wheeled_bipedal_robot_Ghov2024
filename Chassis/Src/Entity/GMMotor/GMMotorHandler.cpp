#include "GMMotorHandler.hpp"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

// 电机数据转换因子
const float GMMotorHandler::RawPosToRad = 0.0007669903939f;   // （7.6699039394282061485904379474597e-4）位置值转换为弧度的转换因子，编码器为十三位，2^13 = 8192, 2 * PI / 8192 (rad)，这样，当编码器的值增加或减少 1 时，它表示电机轴旋转了 2 * PI / 8192 (rad)
const float GMMotorHandler::RawRpmToRadps = 0.1047197551196f; // 0.1047197551f;  // RPM 到弧度每秒的转换因子, 2 * PI / 60 (s)

/**
 * @brief 构造函数，将所有值初始化
 */
GMMotorHandler::GMMotorHandler()
{
    for (int i = 0; i < 8; i++)
        GMMotorList[0][i] = nullptr;
    for (int i = 0; i < 8; i++)
        GMMotorList[1][i] = nullptr;

    for (int i = 0; i < 8; i++)
        can1_send_data_0[i] = 0;
    for (int i = 0; i < 8; i++)
        can1_send_data_1[i] = 0;
    for (int i = 0; i < 8; i++)
        can2_send_data_0[i] = 0;
    for (int i = 0; i < 8; i++)
        can2_send_data_1[i] = 0;
}

GMMotorHandler::~GMMotorHandler()
{
}

/**
 * @brief 注册电机，将电机指针存入MotorList中
 * @param motor 电机指针
 * @param hcan CAN句柄
 * @param canId 电机ID
 */
void GMMotorHandler::registerMotor(GMMotor *GMmotor, CAN_HandleTypeDef *hcan, uint16_t canId)
{
    GMmotor->canId = canId;
    GMmotor->hcan = hcan;

    if (canId >= 0x201 && canId <= 0x208)
    {
        if (GMmotor->hcan == &hcan1)
        {
            GMMotorList[0][canId - 0x201] = GMmotor; // 将电机指针存入MotorList中
            // 判断电机的控制标识符，用于判断是否需要发送控制数据
            if (canId >= 0x201 && canId <= 0x204)
            {
                CAN1_0x200_Exist = true;
            }
            else if (canId >= 0x205 && canId <= 0x208)
            {
                CAN1_0x1FF_Exist = true;
            }
            return;
        }
        else if (GMmotor->hcan == &hcan2)
        {
            GMMotorList[1][canId - 0x201] = GMmotor; // 将电机指针存入MotorList中
            // 判断电机的控制标识符，用于判断是否需要发送控制数据
            if (canId >= 0x201 && canId <= 0x204)
            {
                CAN2_0x200_Exist = true;
            }
            else if (canId >= 0x205 && canId <= 0x208)
            {
                CAN2_0x1FF_Exist = true;
            }
        }
    }
}

void GMMotorHandler::processRawData(CAN_HandleTypeDef *hcan, uint8_t *rx_data, int index)
{
    if (hcan == &hcan1)
    {
        for (int i = 0; i < 8; i++)
        {
            can1_receive_data[index].last_ecd = can1_receive_data[index].ecd;                 ///< 上一次的编码器值
            can1_receive_data[index].ecd = (uint16_t)(rx_data[0] << 8 | rx_data[1]);          ///< 编码器值
            can1_receive_data[index].speed_rpm = (int16_t)(rx_data[2] << 8 | rx_data[3]);     ///< 速度值，单位rpm
            can1_receive_data[index].given_current = (int16_t)(rx_data[4] << 8 | rx_data[5]); ///< 电流值，或者说是转矩值
            can1_receive_data[index].temperate = rx_data[6];                                  ///< 温度值
        }
    }
    else if (hcan == &hcan2)
    {
        for (int i = 0; i < 8; i++)
        {
            can2_receive_data[index].last_ecd = can2_receive_data[index].ecd;                 ///< 上一次的编码器值
            can2_receive_data[index].ecd = (uint16_t)(rx_data[0] << 8 | rx_data[1]);          ///< 编码器值
            can2_receive_data[index].speed_rpm = (int16_t)(rx_data[2] << 8 | rx_data[3]);     ///< 速度值，单位rpm
            can2_receive_data[index].given_current = (int16_t)(rx_data[4] << 8 | rx_data[5]); ///< 电流值，或者说是转矩值
            can2_receive_data[index].temperate = rx_data[6];                                  ///< 温度值
        }
    }

    updateFeedback();
}

/**
 * @brief 发送控制数据
 * @param hcan1 CAN1句柄
 * @param hcan2 CAN2句柄
 */
void GMMotorHandler::sendControlData()
{
    // 循环遍历所有的电机，将电机的控制数据存入can_send_data中
    //< 如果电机列表的最大数量发生变化，这里的循环次数需要修改！！！
    for (int i = 0; i < 8; i++)
    {
        // 处理挂载在CAN1的电机
        if (GMMotorList[0][i] != nullptr)
        {
            // 0x201-0x204，控制标识符为0x200
            if (GMMotorList[0][i]->canId >= 0x201 && GMMotorList[0][i]->canId <= 0x204)
            {
                int index = (GMMotorList[0][i]->canId - 0x200) * 2;
                can1_send_data_0[index - 2] = GMMotorList[0][i]->currentSet >> 8;
                can1_send_data_0[index - 1] = GMMotorList[0][i]->currentSet;
            }
            // 0x205-0x208，控制标识符为0x1FF
            if (GMMotorList[0][i]->canId >= 0x205 && GMMotorList[0][i]->canId <= 0x208)
            {
                int index = (GMMotorList[0][i]->canId - 0x204) * 2;
                can1_send_data_1[index - 2] = GMMotorList[0][i]->currentSet >> 8;
                can1_send_data_1[index - 1] = GMMotorList[0][i]->currentSet;
            }
        }
        // 处理挂载在CAN2的电机
        if (GMMotorList[1][i] != nullptr)
        {
            // 0x201-0x204，控制标识符为0x200
            if (GMMotorList[1][i]->canId >= 0x201 && GMMotorList[1][i]->canId <= 0x204)
            {
                int index = (GMMotorList[1][i]->canId - 0x200) * 2;
                can2_send_data_0[index - 2] = GMMotorList[1][i]->currentSet >> 8;
                can2_send_data_0[index - 1] = GMMotorList[1][i]->currentSet;
            }
            // 0x205-0x208，控制标识符为0x1FF
            if (GMMotorList[1][i]->canId >= 0x205 && GMMotorList[1][i]->canId <= 0x208)
            {
                int index = (GMMotorList[1][i]->canId - 0x204) * 2;
                can2_send_data_1[index - 2] = GMMotorList[1][i]->currentSet >> 8;
                can2_send_data_1[index - 1] = GMMotorList[1][i]->currentSet;
            }
        }
    }
    // 使用bsp_can中的函数发送数据，只应该发送有效数据，防止堵塞。
    if (CAN1_0x200_Exist)
        can_sendData(&hcan1, 0x200, can1_send_data_0, 8); // 向CAN1发送数据，电机控制报文0x200
    if (CAN1_0x1FF_Exist)
        can_sendData(&hcan1, 0x1FF, can1_send_data_1, 8); // 向CAN1发送数据，电机控制报文0x1FF
    if (CAN2_0x200_Exist)
        can_sendData(&hcan2, 0x200, can2_send_data_0, 8); // 向CAN2发送数据，电机控制报文0x200
    if (CAN2_0x1FF_Exist)
        can_sendData(&hcan2, 0x2FF, can2_send_data_1, 8); // 向CAN2发送数据，电机控制报文0x2FF
}

/**
 * @brief 处理并更新电机反馈数据
 * 将电机的反馈数据存入Motorlist中存在的电机的MotorFeedback中
 * @param hcan1 CAN1句柄
 * @param hcan2 CAN2句柄
 */
void GMMotorHandler::updateFeedback()
{
    //< 如果电机列表的最大数量发生变化，这里的循环次数需要修改！！！
    for (int i = 0; i < 8; i++)
    {
        // 处理CAN1的电机
        if (GMMotorList[0][i] != nullptr)
        {
            GMMotorList[0][i]->motorFeedback.last_ecd = GMMotorList[0][i]->motorFeedback.ecd;                // 更新上一次的编码器值
            GMMotorList[0][i]->motorFeedback.ecd = can1_receive_data[i].ecd;                                 // 更新编码器值
            GMMotorList[0][i]->motorFeedback.speed_rpm = can1_receive_data[i].speed_rpm;                     // 更新速度值，单位rpm
            GMMotorList[0][i]->motorFeedback.lastPositionFdb = GMMotorList[0][i]->motorFeedback.positionFdb; // 更新上一次记录的电机位置，单位rad
            GMMotorList[0][i]->motorFeedback.lastSpeedFdb = GMMotorList[0][i]->motorFeedback.speedFdb;       // 更新上一次记录的电机速度，单位rad/s
            // 位置反馈，转换为弧度，并限幅至-PI到PI
            GMMotorList[0][i]->motorFeedback.positionFdb = (float)(can1_receive_data[i].ecd) * RawPosToRad - Math::Pi; // 位置反馈单位转换，范围[-Π，Π]。对于6020，这里是绝对位置。
            GMMotorList[0][i]->motorFeedback.speedFdb = can1_receive_data[i].speed_rpm * RawRpmToRadps;                // 速度反馈单位转换，单位rad/s

            GMMotorList[0][i]->motorFeedback.currentFdb = can1_receive_data[i].given_current; // 转矩电流反馈，不知道什么单位

            GMMotorList[0][i]->motorFeedback.temperatureFdb = (float)can1_receive_data[i].temperate; // 温度反馈
        }
        // 处理CAN2的电机
        if (GMMotorList[1][i] != nullptr)
        {
            GMMotorList[1][i]->motorFeedback.last_ecd = GMMotorList[1][i]->motorFeedback.ecd;                // 更新上一次的编码器值
            GMMotorList[1][i]->motorFeedback.ecd = can1_receive_data[i].ecd;                                 // 更新编码器值
            GMMotorList[1][i]->motorFeedback.speed_rpm = can1_receive_data[i].speed_rpm;                     // 更新速度值，单位rpm
            GMMotorList[1][i]->motorFeedback.lastPositionFdb = GMMotorList[1][i]->motorFeedback.positionFdb; // 更新上一次记录的电机位置，单位rad
            GMMotorList[1][i]->motorFeedback.lastSpeedFdb = GMMotorList[1][i]->motorFeedback.speedFdb;       // 更新上一次记录的电机速度，单位rad/s
            // 位置反馈，转换为弧度，并限幅至-PI到PI
            GMMotorList[1][i]->motorFeedback.positionFdb = (float)(can2_receive_data[i].ecd) * RawPosToRad - Math::Pi; // 位置反馈单位转换，范围[-Π，Π]。对于6020，这里是绝对位置。
            GMMotorList[1][i]->motorFeedback.speedFdb = can2_receive_data[i].speed_rpm * RawRpmToRadps;                // 速度反馈单位转换，单位rad/s

            GMMotorList[1][i]->motorFeedback.currentFdb = can2_receive_data[i].given_current; // 转矩电流反馈，不知道什么单位

            GMMotorList[1][i]->motorFeedback.temperatureFdb = (float)can2_receive_data[i].temperate; // 温度反馈
        }
    }
}

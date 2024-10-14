#ifndef GMMOTORHANDLER_HPP
#define GMMOTORHANDLER_HPP

#include "main.h"

#include "bsp_can.h"
#include "GMMotor.hpp"
#include "Pid.hpp"
#include "Math.hpp"

#define GM6020_CAN_ID 0x1FF ///< 控制ID范围在0x205-0x208的所有电机，由于GM6020的控制报文通常用0x1FF，所以这里用命名为GM6020_CAN_ID，但实际上这个ID也可以控制GM3508
#define GM3508_CAN_ID 0x200 ///< 控制ID范围在0x201-0x204的所有电机

#ifdef __cplusplus


    /**
     * @brief 电机控制类
     */
    class GMMotorHandler
    {
    public:
        /**
         *@brief 使用指针数组, 2个CAN口，每个CAN口最多8个电机
         */
        GMMotor *GMMotorList[2][8];

        GMMotorHandler();
        ~GMMotorHandler();

        // 电机数据转换因子
        static const float RawPosToRad;   // （7.6699039394282061485904379474597e-4）位置值转换为弧度的转换因子，编码器为十三位，2^13 = 8192, 2 * PI / 8192 (rad)，这样，当编码器的值增加或减少 1 时，它表示电机轴旋转了 2 * PI / 8192 (rad)
        static const float RawRpmToRadps; // 0.1047197551f;  // RPM 到弧度每秒的转换因子, 2 * PI / 60 (s)

        /**
         * @brief CAN1接收数据，经过了初步的处理，索引对应电机ID
         */
        GMMotor::motor_measure_t can1_receive_data[8];
        /**
         * @brief CAN2接收数据，经过了初步的处理，索引对应电机ID
         */
        GMMotor::motor_measure_t can2_receive_data[8];

        uint8_t can1_send_data_0[8]; // CAN1电机控制数据，用于控制0x201-0x204
        uint8_t can1_send_data_1[8]; // CAN1电机控制数据，用于控制0x205-0x208

        uint8_t can2_send_data_0[8]; // CAN2电机控制数据，用于控制0x201-0x204
        uint8_t can2_send_data_1[8]; // CAN2电机控制数据，用于控制0x205-0x208

        // 四个标志位，用于判断是否需要发送控制数据
        bool CAN1_0x200_Exist = false; // CAN1是否存在控制报文为0x200电机
        bool CAN1_0x1FF_Exist = false; // CAN1是否存在控制报文为0x1FF电机
        bool CAN2_0x200_Exist = false; // CAN2是否存在控制报文为0x200电机
        bool CAN2_0x1FF_Exist = false; // CAN2是否存在控制报文为0x1FF电机

        /** 
         * @brief 注册电机，将电机指针存入MotorList中
         * @param motor 电机指针
         * @param hcan CAN句柄
         * @param canId 电机ID
         */
        void registerMotor(GMMotor *GMmotor, CAN_HandleTypeDef *hcan, uint16_t canId); // 使用指针作为参数

        /**
         * @brief 处理来自can回调中原始数据
         * @param rx_data 接收到的原始数据
         * @param hcan CAN句柄
         * @param index 电机索引
         * @note 该函数用于处理CAN接收到的原始数据，将其转换为电机测量数据，储存在can_receive_data中。在最后调用updateFeedback()函数更新电机数据。
         */
        void processRawData(CAN_HandleTypeDef *hcan, uint8_t *rx_data, int index);

        /**
         * @brief 发送控制数据
         * @param hcan1 CAN1句柄
         * @param hcan2 CAN2句柄
         */
        void sendControlData();

        /**
         * @brief 处理并更新电机反馈数据
         * @param hcan1 CAN1句柄
         * @param hcan2 CAN2句柄
         */
        void updateFeedback();

        static GMMotorHandler *instance()
        {
            static GMMotorHandler instance;
            return &instance;
        }
    };


#endif
#endif // GMMOTORHANDLER_HPP

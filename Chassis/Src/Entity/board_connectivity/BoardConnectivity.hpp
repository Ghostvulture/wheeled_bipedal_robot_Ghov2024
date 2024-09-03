#ifndef BOARD_CONNECTIVITY_H
#define BOARD_CONNECTIVITY_H

#include "main.h"

#include "bsp_can.h"
#include "bsp_usart.h"

#include "stdlib.h" // for malloc and free
#include "string.h" // for memcpy

#define BOARD_CONNECTIVITY_STD_ID 0xB0 // 板间通信的ID范围为[0xB1, 0xB8]
#define CHASSIS_REMOTE_CONTROL_ID 0xB1 ///< 底盘控制数据在boardMemory中的ID

#ifdef __cplusplus
extern "C"
{
#endif
    class BoardConnectivity
    {
    public:
        BoardConnectivity();
        ~BoardConnectivity();

        /**
         * @enum BoardConnectivityType
         * @brief 板间通信发送通道类型。
         * @note 用于区分板间通信的发送通道。
         * @param BOARD_CONNECTIVITY_CAN_1 CAN1通道
         * @param BOARD_CONNECTIVITY_CAN_2 CAN2通道
         * @param BOARD_CONNECTIVITY_USART USART通道，暂时未实现，用于裁判系统的通信
         *
         * @todo 实现USART通道的通信
         */
        typedef enum
        {
            BOARD_CONNECTIVITY_CAN_1 = 0,
            BOARD_CONNECTIVITY_CAN_2 = 1,
            BOARD_CONNECTIVITY_USART = 3,
        } BoardConnectivityType;

        /**
         * @enum BoardConnectivityDirection
         * @brief 板间通信的方向。
         * @param BOARD_CONNECTIVITY_SEND 此数据要被发送
         * @param BOARD_CONNECTIVITY_RECEIVE 此数据是被接收的数据
         */
        typedef enum
        {
            BOARD_CONNECTIVITY_SEND = 0,
            BOARD_CONNECTIVITY_RECEIVE = 1,
        } BoardConnectivityDirection;

        /**
         * @struct BoardMsg
         * @brief 板间通信数据结构体。
         * @note 用于存储板间通信的数据，包括ID、数据和数据长度
         * @param type 板间通信的类型。
         * @param id 板间通信的ID。范围为[0xB1, 0xB8]。
         * @param len 板间通信的数据长度。
         * @param data 板间通信的数据。
         */
        struct BoardMsg
        {
            BoardConnectivityType type;
            uint16_t id;
            uint16_t len; // 数据长度，不可以超过8
            uint8_t *data;
        };

        /**
         * @brief 板间通信数据存储，数组索引对应板间通信的ID。
         * @note 目前最多存储8个板间通信数据，可以考虑是否需要更多。
         */
        BoardMsg BoardMemory_send[8];

        /**
         * @brief 用于存储接收到的板间通信数据。
         */
        BoardMsg BoardMemory_receive[8];

        /**
         * @brief 初始化函数。
         * 将BoardMemory_send和BoardMemory_receive初始化为nullptr。
         */
        void init(void);

        /**
         * @brief 添加数据到板间通信的内存中。
         * @param data 板间通信的数据。
         * @param id 板间通信的ID。
         * @param len 板间通信的数据长度。
         * @param type 板间通信的类型。
         * @param direction 板间通信的方向。
         */
        void BoardConnectivity_Add2Memory(uint8_t *data, uint16_t id, uint16_t len, BoardConnectivityType type, BoardConnectivityDirection direction);

        /**
         * @brief 发送数据。
         * 将BoardMemory_send中的数据发送出去。
         */
        void BoardConnectivity_Send();

        /**
         * @brief 用于将float转换为uint8_t。
         * @param f 需要转换的float。
         * @param byte 转换后的uint8_t。
         */
        void BoardConnectivity_Float2Byte(float f, uint8_t *byte);

        /**
         * @brief 用于将uint8_t转换为float。
         * @param byte 需要转换的uint8_t。
         * @param f 转换后的float的指针。
         */
        void BoardConnectivity_Byte2Float(uint8_t *byte, float *f);

        /**
         * @brief 获取BoardConnectivity的单例。
         */
        static BoardConnectivity *instance(void)
        {
            static BoardConnectivity instance;
            return &instance;
        }
    };

#ifdef __cplusplus
}
#endif

#endif // BOARD_CONNECTIVITY_H

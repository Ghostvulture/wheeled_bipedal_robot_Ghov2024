#ifndef IST8310_HPP
#define IST8310_HPP

#include "bsp_iic.h"
#include "bsp_dwt.h"
extern I2C_HandleTypeDef hi2c3; // 从main.c中引用iic3句柄

/*---------------------------相关宏定义---------------------------*/
#define IST8310_IICx hi2c3 // IIC句柄

#define IST8310_SCL_GPIOx GPIOA      // IIC_SCL
#define IST8310_SCL_GPIOp GPIO_PIN_8 // IIC_SCL

#define IST8310_SDA_GPIOx GPIOC      // IIC_SDA
#define IST8310_SDA_GPIOp GPIO_PIN_9 // IIC_SDA

#define IST8310_RSTN_GPIOx GPIOG      // IST8310_RSTN
#define IST8310_RSTN_GPIOp GPIO_PIN_6 // IST8310_RSTN

#define IST8310_DRDY_GPIOx GPIOG      // IST8310_DRDY
#define IST8310_DRDY_GPIOp GPIO_PIN_3 // IST8310_DRDY

#define IST8310_DATA_READY_BIT 2 //

#define IST8310_IIC_ADDRESS 0x0E // IIC地址

#define MAG_SEN 0.3f // 磁力计灵敏度

#define IST8310_WHO_AM_I 0x00       // WHO_AM_I寄存器
#define IST8310_WHO_AM_I_VALUE 0x10 // WHO_AM_I寄存器的值

#define IST8310_WRITE_REG_NUM 4 // 写寄存器的数量
/*---------------------类定义---------------------*/

/**
 * @struct ist8310_error_t
 * @brief 磁力计错误信息，继承自deamon_error_t
 * 用于判断磁力计是否正常，也用于守护线程的错误处理
 * @param IST8310_CHIP_ID_ERR 磁力计ID错误则为true
 * @param IST8310_CONFIG_ERR 配置写入错误则为true
 * @param IST8310_CONFIG_ERR_CODE 配置写入错误码
 * @param IST8310_INIT_ERR 自检错误则为true
 */
typedef struct ist8310_error_t
{
    bool IST8310_CHIP_ID_ERR = true;        // 磁力计ID错误则为true
    bool IST8310_CONFIG_ERR = true;         // 配置写入错误则为true
    uint8_t IST8310_CONFIG_ERR_CODE = 0x00; // 配置写入错误码
    bool IST8310_INIT_ERR = true;           // 自检错误则为true

} ist8310_error_t;

/**
 * @struct ist8310_data_t
 * @brief 磁力计数据
 * 用于存储磁力计的数据
 * @param status 磁力计状态
 * @param mag 磁力计数据
 */
typedef struct ist8310_data_t
{
    uint8_t status;
    float mag[3];
} ist8310_data_t;

class IST8310
{
public:
    IST8310();
    ~IST8310();

    /**
     * @brief 磁力计自检结果
     * 用于判断磁力计是否正常，也用于守护线程的错误处理
     */
    ist8310_error_t ist8310_selfTest;

    /**
     * @brief 磁力计原始数据
     * 用于存储磁力计的原始数据
     */
    uint8_t ist8310_raw_data[6];

    /**
     * @brief 磁力计数据
     * 将ist8310_raw_data处理后的数据存储在这里
     */
    ist8310_data_t ist8310_data;

    /**
     * @brief 初始化磁力计
     * 这个函数会初始化磁力计，包括复位、写入配置等
     */
    void init();

    /**
     * @brief 写入寄存器
     * 通过bsp_iic.h中的iic_writeData函数再一次进行封装
     */
    void wirteReg(uint8_t addr, uint8_t *pData, uint16_t len);

    /**
     * @brief 读取寄存器
     * 通过bsp_iic.h中的iic_readData函数再一次进行封装
     */
    void readReg(uint8_t addr, uint8_t *pData, uint16_t len);

    /**
     * @brief 获取磁力计数据
     * 这个函数会读取磁力计的数据，并将数据存储到ist8310_data.mag中
     * 应该在GPIO的中断中调用这个函数
     */
    void getMagData();

    static IST8310 *instance()
    {
        static IST8310 instance;
        return &instance;
    }
};
#endif

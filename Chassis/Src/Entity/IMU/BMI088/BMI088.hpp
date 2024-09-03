#ifndef BMI088_HPP
#define BMI088_HPP

#include "main.h"
#include "bsp_spi.h"
#include "bsp_dwt.h"
#include "BMI088reg.h"
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "PID.hpp"
#include "bsp_pwm.h"

extern SPI_HandleTypeDef hspi1; // 从main.c中引用spi1句柄
extern TIM_HandleTypeDef htim10;

/*---------------------------相关宏定义---------------------------*/
#define BMI088_SPI hspi1 //< SPI句柄

#define BMI088_ACC_GPIOx GPIOA      //< 加速度计片选端口
#define BMI088_ACC_GPIOp GPIO_PIN_4 //< 加速度计片选引脚

#define BMI088_GYRO_GPIOx GPIOB      //< 陀螺仪片选端口
#define BMI088_GYRO_GPIOp GPIO_PIN_0 //< 陀螺仪片选引脚

#define HEATING_RESISTANCE_TIM htim10            //< 加热电阻定时器
#define HEATING_RESISTANCE_CHANNEL TIM_CHANNEL_1 //< 加热电阻通道

/*---------------------------相关结构体---------------------------*/

/**
 * @struct acc_data_t
 * @brief 加速度计数据结构体
 * 存储加速度计的数据
 * @param x x轴加速度
 * @param y y轴加速度
 * @param z z轴加速度
 */
typedef struct acc_data_t
{
    float x;
    float y;
    float z;
    float sensor_time;
    float temperature;
} acc_data_t;

/**
 * @struct gyro_data_t
 * @brief 陀螺仪数据结构体
 * 存储陀螺仪的数据
 * @param roll 横滚角
 * @param pitch 俯仰角
 * @param yaw 偏航角
 */
typedef struct gyro_data_t
{
    float roll;  // x轴
    float pitch; // y轴
    float yaw;   // z轴
} gyro_data_t;

/**
 * @struct bmi088_data_t
 * @brief BMI088数据结构体
 * 存储BMI088的数据
 * @param acc_data 加速度计数据
 * @param gyro_data 陀螺仪数据
 */
typedef struct bmi088_data_t
{
    acc_data_t acc_data;   // 加速度计数据
    gyro_data_t gyro_data; // 陀螺仪数据
} bmi088_data_t;

/**
 * @enum BMI088_SENSOR
 * @brief BMI088传感器的片选
 * @param BMI088_CS_ACC 加速度计片选
 * @param BMI088_CS_GYRO 陀螺仪片选
 */
enum BMI088_SENSOR
{
    BMI088_CS_ACC = 0,
    BMI088_CS_GYRO = 1,
};

/**
 * @struct bmi088_error_t
 * @brief BMI088错误信息，继承自deamon_error_t
 * 用于判断BMI088是否正常，也用于守护线程的错误处理
 */
typedef struct bmi088_error_t
{
    bool ACC_CHIP_ID_ERR = true;  // 加速度计ID错误则为true
    bool ACC_DATA_ERR = true;     // 加速度计数据错误则为true
    bool GYRO_CHIP_ID_ERR = true; // 陀螺仪ID错误则为true
    bool GYRO_DATA_ERR = true;    // 陀螺仪数据错误则为true
    bool BMI088_INIT_ERR = true;  // BMI088初始化错误则为true
} bmi088_error_t;

class BMI088
{
public:
    BMI088();
    ~BMI088();

    /**
     * @brief BMI088数据结构体
     * 存储BMI088的数据
     */
    bmi088_data_t bmi088_data;

    /**
     * @brief BMI088错误结构体
     * 用于检查BMI088的错误
     */
    bmi088_error_t bmi088_selfTest;

    Pid TemperaturePid;

    /**
     * @brief 更新函数
     * 用于更新BMI088的数据
     * @todo 考虑当初始化失败是要将数据设置为一个安全值
     */
    void update(void);
    /*---------------------------------------------功能函数---------------------------------------------*/
    /**
     * @brief 写数据到寄存器
     * @param cs 片选
     * @param addr 寄存器地址
     * @param data 数据
     * @param len 数据长度
     */
    void WriteDataToReg(enum BMI088_SENSOR cs, uint8_t addr, uint8_t *data, uint8_t len);

    /**
     * @brief 从寄存器读取数据
     * @param cs 片选
     * @param addr 寄存器地址
     * @param data 数据
     * @param len 数据长度
     */
    void ReadDataFromReg(enum BMI088_SENSOR cs, uint8_t addr, uint8_t *data, uint8_t len);

    /**
     * @brief 控制加热电阻以控制BMI088温度
     * @param target_temp 目标温度，测量得到大概再29-30度
     */
    void TemperatureControl(float target_temp);

    /*------------------------------------------------初始化函数------------------------------------------------*/

    /**
     * @brief BMI088初始化函数
     * 初始化BMI088,写入配置
     * 自检ID
     * 自检数据
     */
    void BMI088_INIT(void);

    /**
     * @brief BMI088配置初始化函数
     * 初始化BMI088配置
     */
    void BMI088_CONF_INIT(void);

    /*------------------------------------------------读取数据函数------------------------------------------------*/

    /**
     * @brief 读取加速度计数据
     * @param data 加速度计数据结构体
     */
    void ReadAccData(acc_data_t *data);

    /**
     * @brief 读取陀螺仪数据
     * @param data 陀螺仪数据结构体
     */
    void ReadGyroData(gyro_data_t *data);

    /**
     * @brief 读取加速度计传感器时间
     * @param time 传感器时间指针
     */
    void ReadAccSensorTime(float *time);

    /**
     * @brief 读取加速度计温度
     * @param temp 温度指针
     */
    void ReadAccTemperature(float *temp);

    /*------------------------------------------------自检函数------------------------------------------------*/

    /**
     * @brief 验证加速度计ID
     * 若不正确则设置错误标志
     */
    void VerifyAccChipID(void);

    /**
     * @brief 验证陀螺仪ID
     * 若不正确则设置错误标志
     */
    void VerifyGyroChipID(void);

    /**
     * @brief 验证加速度计数据
     * @todo 未实现
     */
    void VerifyAccData(void);

    /**
     * @brief 验证陀螺仪数据
     * @todo 未实现
     */
    void VerifyGyroData(void);

    /*------------------------------------------------实例函数------------------------------------------------*/
    /**
     * @brief BMI088实例
     * @return BMI088实例指针
     */
    static BMI088 *instance()
    {
        static BMI088 instance;
        return &instance;
    }
};

#endif // BMI088_HPP

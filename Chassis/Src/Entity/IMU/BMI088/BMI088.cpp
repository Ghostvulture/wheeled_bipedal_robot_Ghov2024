#include "BMI088.hpp"
#include "BMI088reg.h"

BMI088::BMI088() {}

BMI088::~BMI088() {}

void BMI088::BMI088_INIT(void)
{

    BMI088_CONF_INIT(); //< 初始化配置

    VerifyAccChipID();  //< 验证加速度计ID
    VerifyGyroChipID(); //< 验证陀螺仪ID

    VerifyAccData();  //< 验证加速度计数据
    VerifyGyroData(); //< 验证陀螺仪数据

    TemperaturePid.Init(); //< 温度PID初始化
    TemperaturePid.mode = Pid::PID_POSITION;
    TemperaturePid.kp = 500.0f;
    TemperaturePid.ki = 0.01f;
    TemperaturePid.kd = 0.0f;
    TemperaturePid.maxOut = 999.0f;
    TemperaturePid.maxIOut = 1.0f;

    PWM_Start(&HEATING_RESISTANCE_TIM, TIM_CHANNEL_1); //< 启动加热电阻PWM

    bmi088_selfTest.BMI088_INIT_ERR = false; //< 初始化成功,错误标志设置为false
}

void BMI088::TemperatureControl(float target_temp)
{
    TemperaturePid.ref = target_temp;
    TemperaturePid.fdb = bmi088_data.acc_data.temperature;
    TemperaturePid.UpdateResult();

    if (TemperaturePid.result < 0)
    {
        PWM_SetDutyRatio(&HEATING_RESISTANCE_TIM, 0, TIM_CHANNEL_1);
    }
    else
    {
        PWM_SetDutyRatio(&HEATING_RESISTANCE_TIM, TemperaturePid.result / 999, TIM_CHANNEL_1);
    }
}

void BMI088::VerifyAccChipID()
{
    uint8_t pRxData[2]; //< 读取两个字节,第一个字节是dummy data,第二个字节是chip id

    ReadDataFromReg(BMI088_CS_ACC, ACC_CHIP_ID_ADDR, pRxData, 2); //< 读取加速度计chip id
    //< 如果chip id不等于预设值,则加速度计ID错误,初始化错误
    if (pRxData[1] != ACC_CHIP_ID_VAL)
    {
        bmi088_selfTest.ACC_CHIP_ID_ERR = true;
        bmi088_selfTest.BMI088_INIT_ERR = true;
    }
    else if (pRxData[1] == ACC_CHIP_ID_VAL)
    {
        bmi088_selfTest.ACC_CHIP_ID_ERR = false;
    }
}

void BMI088::VerifyGyroChipID()
{
    uint8_t pRxData;                                                 //< 读取一个字节,chip id
    ReadDataFromReg(BMI088_CS_GYRO, GYRO_CHIP_ID_ADDR, &pRxData, 1); //< 读取陀螺仪chip id
    //< 如果chip id不等于预设值,则陀螺仪ID错误,初始化错误
    if (pRxData != GYRO_CHIP_ID_VAL)
    {
        bmi088_selfTest.GYRO_CHIP_ID_ERR = true;
        bmi088_selfTest.BMI088_INIT_ERR = true;
    }
    else if (pRxData == GYRO_CHIP_ID_VAL)
    {
        bmi088_selfTest.GYRO_CHIP_ID_ERR = false;
    }
}

void BMI088::VerifyAccData() {}

void BMI088::VerifyGyroData() {}

void BMI088::BMI088_CONF_INIT(void)
{
    //< 加速度计初始化
    //< 先软重启，清空所有寄存器
    uint8_t pTxData;
    pTxData = ACC_SOFTRESET_ADDR;
    WriteDataToReg(BMI088_CS_ACC, ACC_SOFTRESET_ADDR, &pTxData, 1);
    DWT_Delay(0.050); //< 延时50ms,重启需要时间

    //< 打开加速度计电源
    pTxData = ACC_PWR_CTRL_ON;
    WriteDataToReg(BMI088_CS_ACC, ACC_PWR_CTRL_ADDR, &pTxData, 1);

    //< 加速度计变成正常模式
    pTxData = ACC_PWR_CONF_ACT;
    WriteDataToReg(BMI088_CS_ACC, ACC_PWR_CONF_ADDR, &pTxData, 1);

    //< 陀螺仪初始化
    //< 先软重启，清空所有寄存器
    pTxData = GYRO_SOFTRESET_ADDR;
    WriteDataToReg(BMI088_CS_GYRO, GYRO_SOFTRESET_ADDR, &pTxData, 1);
    DWT_Delay(0.050); //< 延时50ms,重启需要时间

    //< 陀螺仪变成正常模式
    pTxData = GYRO_LPM1_NOR;
    WriteDataToReg(BMI088_CS_GYRO, GYRO_LPM1_ADDR, &pTxData, 1);

    //< 加速度计配置写入
    //< 写入范围，+-3g的测量范围
    pTxData = ACC_RANGE_3G;
    WriteDataToReg(BMI088_CS_ACC, ACC_RANGE_ADDR, &pTxData, 1);

    //< 写入配置，正常带宽，1600hz输出频率
    pTxData = (ACC_CONF_RESERVED << 7) | (ACC_CONF_BWP_NORM << 6) | (ACC_CONF_ODR_1600_Hz);
    WriteDataToReg(BMI088_CS_ACC, ACC_CONF_ADDR, &pTxData, 1);

    //< 陀螺仪配置写入
    //< 写入范围，+-500°/s的测量范围
    pTxData = GYRO_RANGE_500_DEG_S;
    WriteDataToReg(BMI088_CS_GYRO, GYRO_RANGE_ADDR, &pTxData, 1);

    //< 写入带宽，2000Hz输出频率，532Hz滤波器带宽
    pTxData = GYRO_ODR_2000Hz_BANDWIDTH_532Hz;
    WriteDataToReg(BMI088_CS_GYRO, GYRO_BANDWIDTH_ADDR, &pTxData, 1);
}

void BMI088::ReadAccData(acc_data_t *data)
{
    uint8_t buf[ACC_XYZ_LEN + 1], range;                                  //< 读取数据缓存, range其实是写入的配置
    int16_t acc[3];                                                       //< 加速度计数据暂存
    ReadDataFromReg(BMI088_CS_ACC, ACC_RANGE_ADDR, &range, 1);            //< 读取加速度计配置
    ReadDataFromReg(BMI088_CS_ACC, ACC_X_LSB_ADDR, buf, ACC_XYZ_LEN + 1); //< 读取加速度计数据
    //< 拼接和转换数据
    acc[0] = ((int16_t)buf[1 + 1] << 8) + (int16_t)buf[0 + 1];
    acc[1] = ((int16_t)buf[3 + 1] << 8) + (int16_t)buf[2 + 1];
    acc[2] = ((int16_t)buf[5 + 1] << 8) + (int16_t)buf[4 + 1];
    data->x = (float)acc[0] * BMI088_ACCEL_3G_SEN;
    data->y = (float)acc[1] * BMI088_ACCEL_3G_SEN;
    data->z = (float)acc[2] * BMI088_ACCEL_3G_SEN;
}

void BMI088::ReadGyroData(gyro_data_t *data)
{
    uint8_t buf[GYRO_XYZ_LEN], range; //< 读取数据缓存, range其实是写入的配置
    int16_t gyro[3];
    float unit;
    //< 读取陀螺仪配置
    ReadDataFromReg(BMI088_CS_GYRO, GYRO_RANGE_ADDR, &range, 1);
    switch (range)
    {
    case 0x00:
        unit = 16.384;
        break;
    case 0x01:
        unit = 32.768;
        break;
    case 0x02:
        unit = 65.536;
        break;
    case 0x03:
        unit = 131.072;
        break;
    case 0x04:
        unit = 262.144;
        break;
    default:
        unit = 16.384;
        break;
    }
    //< 读取陀螺仪数据
    ReadDataFromReg(BMI088_CS_GYRO, GYRO_RATE_X_LSB_ADDR, buf, GYRO_XYZ_LEN);
    //< 拼接和转换数据
    gyro[0] = ((int16_t)buf[1] << 8) + (int16_t)buf[0];
    gyro[1] = ((int16_t)buf[3] << 8) + (int16_t)buf[2];
    gyro[2] = ((int16_t)buf[5] << 8) + (int16_t)buf[4];
    data->roll = (float)gyro[0] / unit * DEG2SEC;
    data->pitch = (float)gyro[1] / unit * DEG2SEC;
    data->yaw = (float)gyro[2] / unit * DEG2SEC;
}

void BMI088::ReadAccSensorTime(float *time) // 未完成
{
    uint8_t buf[SENSORTIME_LEN + 1];
    ReadDataFromReg(BMI088_CS_ACC, SENSORTIME_0_ADDR, buf, SENSORTIME_LEN + 1);
    *time = buf[0 + 1] * SENSORTIME_0_UNIT + buf[1 + 1] * SENSORTIME_1_UNIT + buf[2 + 1] * SENSORTIME_2_UNIT;
}

void BMI088::ReadAccTemperature(float *temp) // 未完成
{
    uint8_t buf[TEMP_LEN + 1];
    ReadDataFromReg(BMI088_CS_ACC, TEMP_MSB_ADDR, buf, TEMP_LEN + 1);
    uint16_t temp_uint11 = (buf[0 + 1] << 3) + (buf[1 + 1] >> 5);
    int16_t temp_int11;
    if (temp_uint11 > 1023)
    {
        temp_int11 = (int16_t)temp_uint11 - 2048;
    }
    else
    {
        temp_int11 = (int16_t)temp_uint11;
    }
    *temp = temp_int11 * TEMP_UNIT + TEMP_BIAS;
}


void BMI088::update(void)
{
    if (bmi088_selfTest.BMI088_INIT_ERR == false) // 如果初始化成功则更新数据
    {
        ReadAccData(&bmi088_data.acc_data);
        ReadGyroData(&bmi088_data.gyro_data);
        // ReadAccSensorTime(&bmi088_data.acc_data.sensor_time);
        ReadAccTemperature(&bmi088_data.acc_data.temperature);
        TemperatureControl(40.0f);
    }
}

void BMI088::WriteDataToReg(enum BMI088_SENSOR cs, uint8_t addr, uint8_t *data, uint8_t len)
{
    //< 片选，考虑以后进行bsp_gpio封装
    if (cs == BMI088_CS_ACC)
        HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_RESET);
    else if (cs == BMI088_CS_GYRO)
        HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_RESET);

    uint8_t pTxData = (addr & BMI088_SPI_WRITE_CODE); //< 处理地址为写地址

    spi_sendData(&BMI088_SPI, &pTxData, 1, SPI_BLOCK_MODE); //< 发送地址
    spi_sendData(&BMI088_SPI, data, len, SPI_BLOCK_MODE);   //< 发送数据

    DWT_Delay(0.001);

    //< 取消片选
    if (cs == BMI088_CS_ACC)
        HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_SET);
    else if (cs == BMI088_CS_GYRO)
        HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_SET);
}

void BMI088::ReadDataFromReg(enum BMI088_SENSOR cs, uint8_t addr, uint8_t *data, uint8_t len)
{
    //< 片选，考虑以后进行bsp_gpio封装
    if (cs == BMI088_CS_ACC)
        HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_RESET);
    else if (cs == BMI088_CS_GYRO)
        HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_RESET);

    uint8_t pTxData = (addr | BMI088_SPI_READ_CODE); //< 处理地址为读地址

    spi_sendData(&BMI088_SPI, &pTxData, 1, SPI_BLOCK_MODE); //< 发送地址
    spi_readData(&BMI088_SPI, data, len, SPI_BLOCK_MODE);   //< 读取数据

    //< 取消片选
    if (cs == BMI088_CS_ACC)
        HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_SET);
    else if (cs == BMI088_CS_GYRO)
        HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_SET);
}

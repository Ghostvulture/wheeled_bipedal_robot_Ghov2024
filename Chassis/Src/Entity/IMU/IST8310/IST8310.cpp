#include "IST8310.hpp"

IST8310::IST8310()
{
}

IST8310::~IST8310()
{
}

void IST8310::wirteReg(uint8_t addr, uint8_t *pData, uint16_t len)
{
    iic_writeData(&IST8310_IICx, IST8310_IIC_ADDRESS << 1, addr, I2C_MEMADD_SIZE_8BIT, pData, len);
}

void IST8310::readReg(uint8_t addr, uint8_t *pData, uint16_t len)
{
    iic_readData(&IST8310_IICx, IST8310_IIC_ADDRESS << 1, addr, I2C_MEMADD_SIZE_8BIT, pData, len);
}

// the first column:the registers of IST8310. 第一列:IST8310的寄存器
// the second column: the value to be writed to the registers.第二列:需要写入的寄存器值
// the third column: return error value.第三列:返回的错误码
static const uint8_t ist8310_write_reg_data_error[IST8310_WRITE_REG_NUM][3] = {
    {0x0B, 0x08, 0x01},  // enalbe interrupt  and low pin polarity.开启中断，并且设置低电平
    {0x41, 0x09, 0x02},  // average 2 times.平均采样两次
    {0x42, 0xC0, 0x03},  // must be 0xC0. 必须是0xC0
    {0x0A, 0x0B, 0x04}}; // 200Hz output rate.200Hz输出频率

void IST8310::init()
{
    HAL_GPIO_WritePin(IST8310_RSTN_GPIOx, IST8310_RSTN_GPIOp, GPIO_PIN_RESET); // 拉低复位，考虑以后进行bsp_gpio封装
    DWT_Delay(0.050);                                                          // 延时50ms
    HAL_GPIO_WritePin(IST8310_RSTN_GPIOx, IST8310_RSTN_GPIOp, GPIO_PIN_SET);   // 拉高复位，考虑以后进行bsp_gpio封装
    DWT_Delay(0.050);                                                          // 延时50ms

    uint8_t who_am_i = 0;
    readReg(IST8310_WHO_AM_I, &who_am_i, 1); // 读取WHO_AM_I寄存器
    if (who_am_i != IST8310_WHO_AM_I_VALUE)
    {
        // 磁力计ID错误
        ist8310_selfTest.IST8310_CHIP_ID_ERR = true;
        return;
    }
    for (int wirte_num = 0; wirte_num < IST8310_WRITE_REG_NUM; wirte_num++)
    {
        uint8_t write_data = ist8310_write_reg_data_error[wirte_num][1];
        // iic_writeData(&IST8310_IICx, IST8310_IIC_ADDRESS, ist8310_write_reg_data_error[wirte_num][0], &write_data, 1); // 写入寄存器
        wirteReg(ist8310_write_reg_data_error[wirte_num][0], &write_data, 1);
        DWT_Delay(0.150); // 延时150ms
        uint8_t read_data = 0;
        // iic_readData(&IST8310_IICx, IST8310_IIC_ADDRESS, ist8310_write_reg_data_error[wirte_num][0], &read_data, 1); // 读取寄存器
        readReg(ist8310_write_reg_data_error[wirte_num][0], &read_data, 1);
        if (read_data != ist8310_write_reg_data_error[wirte_num][1])
        {
            // 写入寄存器失败
            ist8310_selfTest.IST8310_CONFIG_ERR = true;
            ist8310_selfTest.IST8310_CONFIG_ERR_CODE = ist8310_write_reg_data_error[wirte_num][2];
            return;
        }
    }
    ist8310_selfTest.IST8310_INIT_ERR = false;
}

void IST8310::getMagData()
{
    int16_t temp_ist8310_data = 0;

    readReg(0x03, ist8310_raw_data, 6); // 读取磁力计数据
    // 进行数据校验和拼接
    if (ist8310_raw_data[0] & 0x01)
    {
        temp_ist8310_data = (int16_t)((ist8310_raw_data[1] << 8) | ist8310_raw_data[0]);
        ist8310_data.mag[0] = MAG_SEN * temp_ist8310_data;
        temp_ist8310_data = (int16_t)((ist8310_raw_data[3] << 8) | ist8310_raw_data[2]);
        ist8310_data.mag[1] = MAG_SEN * temp_ist8310_data;
        temp_ist8310_data = (int16_t)((ist8310_raw_data[5] << 8) | ist8310_raw_data[4]);
        ist8310_data.mag[2] = MAG_SEN * temp_ist8310_data;
    }
    else
    {
        ist8310_data.status &= ~(1 << IST8310_DATA_READY_BIT);
    }
}

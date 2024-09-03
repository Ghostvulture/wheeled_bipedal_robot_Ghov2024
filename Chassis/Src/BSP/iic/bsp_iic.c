#include "bsp_iic.h"

void iic_init()
{
}

void iic_writeData(I2C_HandleTypeDef *hi2c, uint16_t dev_addr, uint16_t reg_addr, uint16_t mem_addsize, uint8_t *pData, uint16_t len)
{
    HAL_I2C_Mem_Write(hi2c, dev_addr, reg_addr, mem_addsize, pData, len, 100);
}

void iic_readData(I2C_HandleTypeDef *hi2c, uint16_t dev_addr, uint16_t reg_addr, uint16_t mem_addsize, uint8_t *pData, uint16_t len)
{

    HAL_I2C_Mem_Read(hi2c, dev_addr, reg_addr, mem_addsize, pData, len, 100);
}

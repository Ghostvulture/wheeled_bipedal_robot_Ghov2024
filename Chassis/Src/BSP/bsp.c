#include "bsp.h"

void bsp_init(void)
{
    can_filter_init(); // CAN滤波器初始化
    DWT_Init(168);     // 168MHz，c板主频168MHz
    gpio_init();       // GPIO初始化，主要是将LED，SPI，IIC所需要的引脚初始化
}

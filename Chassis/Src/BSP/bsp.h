#ifndef _BSP_H
#define _BSP_H

#include "bsp_gpio.h"
#include "bsp_usart.h"
#include "bsp_can.h"
#include "bsp_dwt.h"

#ifdef __cplusplus
extern "C"
{
#endif
	
void bsp_init(void);

	
#ifdef __cplusplus
}
#endif
#endif // !_BSP_H

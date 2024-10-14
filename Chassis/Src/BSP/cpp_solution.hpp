#ifndef CPP_SOLUTION_HPP
#define CPP_SOLUTION_HPP

// #include "BoardConnectivity.hpp"

#include "bsp_can.h"

#ifdef __cplusplus
extern "C"
{
#endif
	
void can_callback_cpp(CAN_HandleTypeDef *hcan);

void ahrs_update();

void ist8310_getMagData();

	
#ifdef __cplusplus
}
#endif
#endif // CPP_SOLUTION_HPP
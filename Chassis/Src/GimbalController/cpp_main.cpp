#include "ChassisController.hpp"
#include "GimbalController.hpp"
#include "ShooterController.hpp"
#include "BalanceController.hpp"

#include "GMMotorHandler.hpp"
#include "LKMotorHandler.hpp"

#include "Dr16.hpp"
#include "BoardConnectivity.hpp"
#include "BMI088.hpp"
#include "IST8310.hpp"
#include "LED.h"
#include "AHRS.hpp"

#include "main.h"

#include "bsp.h"
#include "bsp_can.h"
#include "bsp_tim.h"

#include "cpp_main.hpp"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern TIM_HandleTypeDef htim2;

GMMotorHandler *GMmotorHandler = GMMotorHandler::instance(); // 电机控制
LKMotorHandler *LKmotorHandler = LKMotorHandler::instance(); 

Dr16 *dr16 = Dr16::instance();
BMI088 *bmi088 = BMI088::instance();
IST8310 *ist8310 = IST8310::instance();
BoardConnectivity *boardConnectivity = BoardConnectivity::instance();
AHRS *ahrs = AHRS::instance(); 

ChassisController *chassisController = ChassisController::instance();
GimbalController *gimbalController = GimbalController::instance();
ShooterController *shooterController = ShooterController::instance();

BalanceController *balanceController = BalanceController::instance();


void main_demo(){

  bsp_init();

  bmi088->BMI088_INIT();     
  ist8310->init();
	ahrs->INS_Init();
	HAL_TIM_Base_Start_IT(&htim2);
	
	boardConnectivity->init();
  remote_control_init();     

  chassisController->init(); 
  gimbalController->init();
	shooterController->init();
  balanceController->init();

}


/**
 * @brief  系统中断，每1ms进入�?�?
 * 代码的主循环
 */
void SysTick_Handler(void)
{
  HAL_IncTick();
  // receive and update data
  // run control loop
  dr16->updateData();
  bmi088->update();
	
  // LKmotorHandler->updateFeedback();

	// chassisController->run();
  balanceController->run();
	
//  GMmotorHandler->sendControlData();
	LKmotorHandler->sendControlData(&hcan1, &hcan2);	
}








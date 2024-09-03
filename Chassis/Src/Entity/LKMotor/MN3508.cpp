#include "MN3508.hpp"
#include "bsp_can.h"
#include "math_first_order_filter.h"
/**
 * @brief GM6020类的构造函数。
 * 初始化电机的控制模式、各种设定值和PID控制器。
 */
 FirstOrderFilter mn3508SpdFilter;
 
MN3508::MN3508()
{
    // 初始化为松开模式
    controlMode = RELAX_MODE;

    // 初始化设定值为0
    speedSet = 0;
    positionSet = 0;
    currentSet = 0;
    maxCurrent = 2000; // 电机最大电流设定

    // 初始化电机反馈数据
    motorFeedback.speedFdb = 0;
    motorFeedback.lastSpeedFdb = 0;
    motorFeedback.positionFdb = 0;
    motorFeedback.lastPositionFdb = 0;
    motorFeedback.temperatureFdb = 0;

    // PID控制器初始化
    speedPid.mode = Pid::PID_POSITION;
    speedPid.kp = 0.1;
    speedPid.ki = 0.0;
    speedPid.kd = 0.0;
    speedPid.maxOut = 25000;
    speedPid.maxIOut = 3;

    positionPid.mode = Pid::PID_POSITION;
    positionPid.kp = 0.1;
    positionPid.ki = 0.0;
    positionPid.kd = 0.0;
    positionPid.maxOut = 25000;
    positionPid.maxIOut = 3;
		
		mn3508SpdFilter.SetTau(0.1f);       //设置速度滤波器的时间常数
		mn3508SpdFilter.SetUpdatePeriod(1.0f);  //设置速度滤波器的更新周期为1毫秒
}

/**
 * @brief GM6020类的析构函数。
 */
MN3508::~MN3508()
{
}

/**
 * @brief 设置电机输出。
 * 根据当前控制模式，计算并设置电机的当前输出。
 */
void MN3508::setOutput()
{

		
    if (this->controlMode == RELAX_MODE)
    {
        this->currentSet = 0.0; // 松开模式下电流设定为0
				return;
    }
    else if (this->controlMode == SPD_MODE)
    {
        // currentSet = torqueSet / 0.32f / 32.0f * 2000.0f;
        this->currentSet = this->torqueSet * 392.78f;
				this->currentSet = Math::FloatConstrain(currentSet, -2000, 2000);
				return;
				
    } 
    else if (this->controlMode == SPD_MODE)
    {

        // 摩擦轮只用速度环控制，0xA2单电机指令
				this->speedPid.ref = this->speedSet;
				
				mn3508SpdFilter.SetInput(this->motorFeedback.speedFdb);
				mn3508SpdFilter.Update();
			
        this->speedPid.fdb = mn3508SpdFilter.GetResult();
        this->speedPid.UpdateResult();

        this->currentSet = this->speedPid.result; // 根据速度PID结果设置电流
    }

    else
    {
        this->currentSet = 0.0; // 其他情况电流设定为0
				return;
    }

    // 限制电流输出不超过最大值
    this->currentSet = Math::FloatConstrain(this->currentSet, -maxCurrent, maxCurrent);
}




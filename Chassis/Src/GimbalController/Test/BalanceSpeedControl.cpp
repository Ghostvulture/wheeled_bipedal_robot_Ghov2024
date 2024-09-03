#include "BalanceSpeedControl.hpp"

void BalanceSpeedControl::init()
{
    Motor_SpeedSet = 0;

    /*---------------------------初始化PID---------------------------*/
    LK9025SpeedPid.kp = 150.0f;
    LK9025SpeedPid.ki = 0.0f;
    LK9025SpeedPid.kd = 0.0f;
    LK9025SpeedPid.maxIOut = 3;
    LK9025SpeedPid.maxOut = 15000;

    /*-----------------------------------------左右轮分别初始化-----------------------------------------*/
    LMotor->controlMode = LK9025::POS_MODE; 
    LMotor->speedPid = LK9025SpeedPid;
    LMotor->speedPid.Clear();
    LMotor->positionPid.Clear();

    RMotor->controlMode = LK9025::POS_MODE; 
    RMotor->speedPid = LK9025SpeedPid;
    RMotor->speedPid.Clear();
    RMotor->positionPid.Clear();


    LMotor->speedSet = 0;
    RMotor->speedSet = 0;

    LMotor->setOutput();
    RMotor->setOutput();

}

void BalanceSpeedControl::enter()
{
    // 接受遥控器输入
    Motor_SpeedSet += Dr16::instance()->rc_right_x * 0.008f;

}

void BalanceSpeedControl::execute()
{

    // 速度环控制
    LMotor->speedSet = Motor_SpeedSet;
    LMotor->speedPid.ref = LMotor->speedSet;
    LMotor->speedPid.fdb = LMotor->motorFeedback.speedFdb;
    LMotor->speedPid.UpdateResult();

    LMotor->currentSet = LMotor->speedPid.result; // 根据速度PID结果设置电流

}

void BalanceSpeedControl::exit()
{
}

void BalanceSpeedControl::run()
{
    enter();
    execute();
    exit();
}
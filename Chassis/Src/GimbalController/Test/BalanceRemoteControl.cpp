#include "BalanceRemoteControl.hpp"

const float DegreeToRad = Math::Pi / 180.0f;

void BalanceRemoteControl::init()
{
    // 初始化电机位置
    Motor_PositionSet = 0;

    /*---------------------------初始化PID---------------------------*/
    LK9025SpeedPid.kp = 150.0f;
    LK9025SpeedPid.ki = 0.0f;
    LK9025SpeedPid.kd = 0.0f;
    LK9025SpeedPid.maxIOut = 3;
    LK9025SpeedPid.maxOut = 15000;

    LK9025PositionPid.kp = 150.0f;
    LK9025PositionPid.ki = 0.0f;
    LK9025PositionPid.kd = 0.0f;
    LK9025PositionPid.maxIOut = 3;
    LK9025PositionPid.maxOut = 15000;

    LK8016PositionPid.kp = 150.0f;
    LK8016PositionPid.ki = 0.0f;
    LK8016PositionPid.kd = 0.0f;
    LK8016PositionPid.maxIOut = 3;
    LK8016PositionPid.maxOut = 15000;

    /*-----------------------------------------左右轮分别初始化-----------------------------------------*/
    LMotor->controlMode = LK9025::POS_MODE; 
    LMotor->speedPid = LK9025SpeedPid;
    LMotor->positionPid = LK9025PositionPid;
    LMotor->speedPid.Clear();
    LMotor->positionPid.Clear();

    RMotor->controlMode = LK9025::POS_MODE; 
    RMotor->speedPid = LK9025SpeedPid;
    RMotor->positionPid = LK9025PositionPid;
    RMotor->speedPid.Clear();
    RMotor->positionPid.Clear();

    RD->controlMode = LK9025::POS_MODE; 
    RD->speedPid = LK8016PositionPid;
    RD->positionPid = LK8016PositionPid;
    RD->speedPid.Clear();
    RD->positionPid.Clear();
    


    RD->positionSet = 0;

    LMotor->controlMode = LK9025::RELAX_MODE;
    RMotor->controlMode = LK9025::RELAX_MODE;

    LMotor->setOutput();
    RMotor->setOutput();
    RD->setOutput();

}

void BalanceRemoteControl::enter()
{
    // 接受遥控器输入
    Motor_PositionSet += Dr16::instance()->rc_left_x * 0.008f; 
    //给遥控器赋值增加限幅，防止输出>=1.5Pi超出量程
    if (Motor_PositionSet >= Math::Pi)
    {
        Motor_PositionSet -= Math::PiX2;
    }
    else if (Motor_PositionSet <= -Math::Pi)
    {
        Motor_PositionSet += Math::PiX2;
    }
		RD->positionSet = 0.43;

}

void BalanceRemoteControl::execute()
{


    // // 外环控制，位置环控制
    // LMotor->positionSet = RMotor->positionSet = Motor_PositionSet;

    // LMotor->positionPid.ref = LMotor->positionSet;
    // RMotor->positionPid.ref = RMotor->positionSet;

    // LMotor->positionPid.fdb = LMotor->motorFeedback.positionFdb;
    // RMotor->positionPid.fdb = RMotor->motorFeedback.positionFdb;



    // LMotor->positionPid.UpdateResult();
    // RMotor->positionPid.UpdateResult();

    // // 内环控制，速度环控制
    // LMotor->speedPid.ref = LMotor->positionPid.result;
    // RMotor->speedPid.ref = RMotor->positionPid.result;

    // LMotor->speedPid.fdb = LMotor->motorFeedback.speedFdb;
    // RMotor->speedPid.fdb = RMotor->motorFeedback.speedFdb;

    // LMotor->speedPid.UpdateResult();
    // RMotor->speedPid.UpdateResult();

    // LMotor->currentSet = LMotor->speedPid.result; // 根据速度PID结果设置电流
    // RMotor->currentSet = RMotor->speedPid.result;

    RD->positionPid.ref = RD->positionSet;
    RD->positionPid.fdb = RD->motorFeedback.positionFdb;
    RD->positionPid.UpdateResult();
    RD->currentSet = RD->positionPid.result;

}

void BalanceRemoteControl::exit()
{
}

void BalanceRemoteControl::run()
{
    enter();
    execute();
    exit();
}

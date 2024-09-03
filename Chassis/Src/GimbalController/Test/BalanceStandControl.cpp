#include "BalanceStandControl.hpp"

void BalanceStandControl::init()
{
    Motor_angleSet = 0;
    Motor_omegaSet = 0;

    /*---------------------------初始化PID---------------------------*/
    LK9025AnglePid.kp = 20.0f;
    LK9025AnglePid.ki = 0.0f;
    LK9025AnglePid.kd = 0.0f;
    LK9025AnglePid.maxIOut = 3;
    LK9025AnglePid.maxOut = 15000;

    LK9025OmegaPid.kp = 20.0f;
    LK9025OmegaPid.ki = 0.0f;
    LK9025OmegaPid.kd = 0.0f;
    LK9025OmegaPid.maxIOut = 3;
    LK9025OmegaPid.maxOut = 15000;

    /*-----------------------------------------左右轮分别初始化-----------------------------------------*/
    LMotor->controlMode = LK9025::POS_MODE; 
    LMotor->anglePid = LK9025AnglePid;
    LMotor->omegaPid = LK9025OmegaPid;
    LMotor->speedPid.Clear();
    LMotor->positionPid.Clear();
    LMotor->anglePid.Clear();
    LMotor->omegaPid.Clear();

    RMotor->controlMode = LK9025::POS_MODE; 
    RMotor->anglePid = LK9025AnglePid;
    RMotor->omegaPid = LK9025OmegaPid;
    RMotor->speedPid.Clear();
    RMotor->positionPid.Clear();
    RMotor->anglePid.Clear();
    RMotor->omegaPid.Clear();
    
    LMotor->positionSet = 0;
    RMotor->positionSet = 0;

    LMotor->setOutput();
    RMotor->setOutput();

}

void BalanceStandControl::enter()
{
    //从AHS系统中获取roll倾角和角速度
    Motor_angleRef = AHRS::instance()->INS.Roll;
    Motor_omegaRef = AHRS::instance()->INS.Gyro[1];
}

void BalanceStandControl::execute()
{
    // 角度环控制
    LMotor->anglePid.ref = RMotor->anglePid.ref = Motor_angleSet;
    LMotor->anglePid.fdb = RMotor->anglePid.fdb = Motor_angleRef;
    LMotor->anglePid.UpdateResult();
    RMotor->anglePid.UpdateResult();

    // 角速度环控制
    LMotor->omegaPid.ref = Motor_omegaSet;
    RMotor->omegaPid.ref = Motor_omegaSet;

    LMotor->omegaPid.fdb = Motor_omegaRef;
    RMotor->omegaPid.fdb = Motor_omegaRef;

    LMotor->omegaPid.UpdateResult();
    RMotor->omegaPid.UpdateResult();

    LMotor->currentSet = LMotor->anglePid.result + LMotor->omegaPid.result; // 根据角度PID结果和角速度PID结果设置电流
    RMotor->currentSet = RMotor->anglePid.result + RMotor->omegaPid.result;
}

void BalanceStandControl::exit()
{
}

void BalanceStandControl::run()
{
    enter();
    execute();
    exit();
}
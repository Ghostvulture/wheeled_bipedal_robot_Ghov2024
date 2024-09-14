#include "BalanceStandControl.hpp"
#include "odometer.hpp"

void BalanceStandControl::init()
{
    Motor_angleSet = 0;
    Motor_omegaSet = 0;
    Motor_positionSet = 0;

    /*---------------------------初始化直立环PID---------------------------*/
    //角度环
    LK9025AnglePid.kp = 60.0f;
    LK9025AnglePid.ki = 0.0f;
    LK9025AnglePid.kd = 5.0f;
    LK9025AnglePid.maxIOut = 3;
    LK9025AnglePid.maxOut = 15000;
    //角速度环
    LK9025OmegaPid.kp = 30.0f;
    LK9025OmegaPid.ki = 0.0f;
    LK9025OmegaPid.kd = 0.0f;
    LK9025OmegaPid.maxIOut = 3;
    LK9025OmegaPid.maxOut = 15000;

    /*---------------------------初始化行进环PID---------------------------*/
    //位移环
    LK9025PositionPid.kp = 60.0f;
    LK9025PositionPid.ki = 0.0f;
    LK9025PositionPid.kd = 0.0f;
    LK9025PositionPid.maxIOut = 3;
    LK9025PositionPid.maxOut = 15000;
    //速度环
    LK9025SpeedPid.kp = 60.0f;
    LK9025SpeedPid.ki = 0.0f;
    LK9025SpeedPid.kd = 0.0f;
    LK9025SpeedPid.maxIOut = 3;
    LK9025SpeedPid.maxOut = 15000;
    /*-----------------------------------------左右轮分别初始化-----------------------------------------*/
    LMotor->controlMode = LK9025::POS_MODE; 
    LMotor->anglePid = LK9025AnglePid;
    LMotor->omegaPid = LK9025OmegaPid;
    LMotor->positionPid = LK9025PositionPid;
    LMotor->speedPid.Clear();
    LMotor->positionPid.Clear();
    LMotor->anglePid.Clear();
    LMotor->omegaPid.Clear();


    RMotor->controlMode = LK9025::POS_MODE; 
    RMotor->anglePid = LK9025AnglePid;
    RMotor->omegaPid = LK9025OmegaPid;
    RMotor->positionPid = LK9025PositionPid;
    RMotor->speedPid.Clear();
    RMotor->positionPid.Clear();
    RMotor->anglePid.Clear();
    RMotor->omegaPid.Clear();
    
    LMotor->positionSet = 0;
    RMotor->positionSet = 0;

    LMotor->setOutput();
    RMotor->setOutput();

    //里程计初始化,进入状态机时初始化一次
    //TODO: 仅仅当Vx趋近于0时开启位移环
    Odometer odemeter(*LMotor, *RMotor);
    Motor_DisplacementRef = odemeter.init();
}

void BalanceStandControl::enter()
{
    //从AHS系统中获取roll倾角和角速度
    Motor_angleRef = AHRS::instance()->INS.Roll;
    Motor_omegaRef = AHRS::instance()->INS.Gyro[1];
    
    //获取遥控器设定速度
    Vx = Dr16::instance()->rc_right_y * 5;

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

    odometer->update();
    //位移环pid，当且仅当Vx趋近于0时开启位移环
    if (Vx < 0.1 && Vx > -0.1)
    {    
    //to test
    // LMotor->positionPid.ref = RMotor->positionPid.ref = Motor_positionRef; 
    // LMotor->positionPid.fdb = LMotor->motorFeedback.positionFdb;
    // RMotor->positionPid.fdb = RMotor->motorFeedback.positionFdb;
    LMotor->positionPid.ref = RMotor->positionPid.ref = Motor_DisplacementRef / 0.1; //轮子半径为0.1m，位移->弧度制转角
    LMotor->positionPid.fdb = RMotor->positionPid.fdb = odometer->odometry.x / 0.1;

    LMotor->positionPid.UpdateResult();
    RMotor->positionPid.UpdateResult();
    }

    //速度环pid
    LMotor->speedPid.ref = RMotor->speedPid.ref = Vx;
    LMotor->speedPid.fdb = LMotor->motorFeedback.speedFdb;
    RMotor->speedPid.fdb = RMotor->motorFeedback.speedFdb;



    LMotor->currentSet = LMotor->anglePid.result + LMotor->omegaPid.result + LMotor->positionPid.result; //设置电流
    RMotor->currentSet = - (RMotor->anglePid.result + RMotor->omegaPid.result + RMotor->positionPid.result); 
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
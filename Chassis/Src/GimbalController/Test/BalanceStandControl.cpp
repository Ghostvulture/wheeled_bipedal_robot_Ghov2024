#include "BalanceStandControl.hpp"

#include "Math.hpp"
#include "arm_math.h"

//float VEL_KF;


static float vel_temp;
//static float q_inv[4];
//static float a_body[4];
static float a_world[4] = {0};
static float tmp[4] = {0};
static float a;
float test_FilteredValue;

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
    LMotor->controlMode = LK9025::TOR_MODE; 
    LMotor->anglePid = LK9025AnglePid;
    LMotor->omegaPid = LK9025OmegaPid;
    LMotor->positionPid = LK9025PositionPid;
    LMotor->speedPid.Clear();
    LMotor->positionPid.Clear();
    LMotor->anglePid.Clear();
    LMotor->omegaPid.Clear();


    RMotor->controlMode = LK9025::TOR_MODE; 
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
    
}

void BalanceStandControl::enter()
{
    //从AHS系统中获取roll倾角和角速度
    Motor_angleRef = AHRS::instance()->INS.Roll;
    Motor_omegaRef = AHRS::instance()->INS.Gyro[1];

    //融合里程计
    vel_temp = 0.5f * (LMotor->motorFeedback.speedFdb + RMotor->motorFeedback.speedFdb) * WHEEL_RADIUS;
    float q_inv[4] = {AHRS::instance()->INS.q[0], -AHRS::instance()->INS.q[1], -AHRS::instance()->INS.q[2], -AHRS::instance()->INS.q[3]};
    float a_body[4] = {AHRS::instance()->INS.Accel[0], AHRS::instance()->INS.Accel[1], AHRS::instance()->INS.Accel[2]};
    arm_quaternion_product_f32(AHRS::instance()->INS.q, a_body, tmp, 1);
    arm_quaternion_product_f32(tmp, q_inv, a_world, 1);
    a = sqrtf(a_world[1] * a_world[1] + a_world[2] * a_world[2]) * 
        arm_cos_f32(atan2f(a_world[2], a_world[1]) - AHRS::instance()->INS.Yaw);//yaw单位？

    kf_vel.UpdateKalman(vel_temp, a);

    odometer_msg.x = kf_vel.GetXhat();
		odometer_msg.v = kf_vel.GetVhat();
    odometer_msg.a_z = a_world[3];

//    VEL_KF = vel_temp;

    // //获取遥控器设定速度
    // Vx = Dr16::instance()->rc_right_y * 0.5f;

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





    LMotor->torqueSet = LMotor->anglePid.result + LMotor->omegaPid.result; //设置电流
    RMotor->torqueSet = - (RMotor->anglePid.result + RMotor->omegaPid.result); 

    LMotor->setOutput();   
    RMotor->setOutput();
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
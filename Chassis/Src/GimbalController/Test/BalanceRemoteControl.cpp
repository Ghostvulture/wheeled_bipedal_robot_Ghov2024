#include "BalanceRemoteControl.hpp"

const float DegreeToRad = Math::Pi / 180.0f;

void BalanceRemoteControl::init()
{
    // 初始化电机位置
    Motor_PositionSet = 0;

    /*---------------------------初始化PID---------------------------*/
    LK9025SpeedPid.kp = 50.0f;
    LK9025SpeedPid.ki = 0.0f;
    LK9025SpeedPid.kd = 5.0f;
    LK9025SpeedPid.maxIOut = 3;
    LK9025SpeedPid.maxOut = 15000;

    LK9025PositionPid.kp = 50.0f;
    LK9025PositionPid.ki = 0.0f;
    LK9025PositionPid.kd = 5.0f;
    LK9025PositionPid.maxIOut = 3;
    LK9025PositionPid.maxOut = 15000;

    LK8016PositionPid.kp = 200.0f;
    LK8016PositionPid.ki = 0.0f;
    LK8016PositionPid.kd = 0.0f;
    LK8016PositionPid.maxIOut = 3;
    LK8016PositionPid.maxOut = 15000;

    legLengthPid.kp = 200.0f;
    legLengthPid.ki = 0.0f;
    legLengthPid.kd = 0.0f;
    legLengthPid.maxIOut = 3;
    legLengthPid.maxOut = 15000;



    /*-----------------------------------------左右轮分别初始化-----------------------------------------*/
    LMotor->controlMode = LK9025::TOR_MODE; 
    LMotor->speedPid = LK9025SpeedPid;
    LMotor->positionPid = LK9025PositionPid;
    LMotor->speedPid.Clear();
    LMotor->positionPid.Clear();

    RMotor->controlMode = LK9025::TOR_MODE; 
    RMotor->speedPid = LK9025SpeedPid;
    RMotor->positionPid = LK9025PositionPid;
    RMotor->speedPid.Clear();
    RMotor->positionPid.Clear();


    RD->controlMode = LK8016::RELAX_MODE;
    RU->controlMode = LK8016::RELAX_MODE;
    LD->controlMode = LK8016::RELAX_MODE;
    LU->controlMode = LK8016::RELAX_MODE;
    RD->setOutput();
    RU->setOutput();
    LD->setOutput();
    LU->setOutput();

    //左右轮的力矩输入初始化，每次进入该状态时更新
    force_torque_left[0] = 0.0f;
    force_torque_left[1] = 0.0f;
    force_torque_right[0] = 0.0f;
    force_torque_right[1] = 0.0f;

    lqr.InitMatX(&mat_target, &mat_observed);

}

void BalanceRemoteControl::enter()
{

    float a_world[4] = {0};
    float tmp[4] = {0};
    //融合里程计,得出观测值
    float vel_temp = 0.5f * (LMotor->motorFeedback.speedFdb + RMotor->motorFeedback.speedFdb) * WHEEL_RADIUS;
    float q_inv[4] = {AHRS::instance()->INS.q[0], -AHRS::instance()->INS.q[1], -AHRS::instance()->INS.q[2], -AHRS::instance()->INS.q[3]};
    float a_body[4] = {AHRS::instance()->INS.Accel[0], AHRS::instance()->INS.Accel[1], AHRS::instance()->INS.Accel[2]};
    arm_quaternion_product_f32(AHRS::instance()->INS.q, a_body, tmp, 1);
    arm_quaternion_product_f32(tmp, q_inv, a_world, 1);
    float a = sqrtf(a_world[1] * a_world[1] + a_world[2] * a_world[2]) * 
    arm_cos_f32(atan2f(a_world[2], a_world[1]) - AHRS::instance()->INS.Yaw);//yaw单位？

    kf_vel.UpdateKalman(vel_temp, a);

    odometer_msg.x = kf_vel.GetXhat();
    odometer_msg.v = kf_vel.GetVhat();
    odometer_msg.a_z = a_world[3];

    //解算双腿距离，小板凳不用
    link_solver[0].Resolve(RD->motorFeedback.positionFdb, RU->motorFeedback.positionFdb);
    link_solver[1].Resolve(LD->motorFeedback.positionFdb, LU->motorFeedback.positionFdb);

    //更新状态变量X观测值，单位统一为弧度制
    observed_x[0] =
            0.5f * (1.57+1.57) + AHRS::instance()->INS.Roll*0.017453293f -
            0.5f * PI;
    observed_x[1] = 0.5f * (0 + 0) + AHRS::instance()->INS.Gyro[1];    //摆杆角速度，小板凳近似于0
    observed_x[2] = odometer_msg.x;
    observed_x[3] = odometer_msg.v;
    observed_x[4] = AHRS::instance()->INS.Roll*0.017453293f;
    observed_x[5] = AHRS::instance()->INS.Gyro[1];

    //更新状态变量X目标值，其中使其原地平衡，位移目标值为x观测值
    target_x[0] = 0.0f;
    target_x[1] = 0.0f;
    target_x[2] = observed_x[2];
    target_x[3] = 0;
    target_x[4] = 0;
    target_x[5] = 0;

    lqr.refreshLQRK(0.11);
    lqr.LQRCal(lqr_out);

    force_torque_left[1] = 0.5f * lqr_out[1];
    force_torque_right[1] = 0.5f * lqr_out[1];

}

void BalanceRemoteControl::execute()
{

    LMotor->torqueSet = -force_torque_left[1];
    RMotor->torqueSet = force_torque_right[1];
    LMotor->setOutput();
    RMotor->setOutput();

    // RD->positionPid.ref = RD->positionSet;
    // RD->positionPid.fdb = RD->motorFeedback.positionFdb;
    // RD->positionPid.UpdateResult();
    // RD->torqueSet = RD->positionPid.result;
    // RD->setOutput();


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

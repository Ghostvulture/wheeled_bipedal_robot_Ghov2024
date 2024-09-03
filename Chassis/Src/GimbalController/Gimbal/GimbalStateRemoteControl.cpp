#include "GimbalStateRemoteControl.hpp"

void GimbalStateRemoteControl::init()
{
    // 新机器人下代码之后需要使用debug查看电机位置并修改下面的值
    PitchLowerLimit = 0.460961252; // Pitch轴电机最小角度
    PitchUpperLimit = 5.97485542;  // Pitch轴电机最大角度

    // 云台Yaw轴电机
    YawMotor->controlMode = GM6020::POS_MODE;
    // 速度环PID参数
    YawMotor->speedPid.kp = 200.0f;
    YawMotor->speedPid.ki = 0.0f;
    YawMotor->speedPid.kd = 0.0f;
    YawMotor->speedPid.maxIOut = 3.0f;
    YawMotor->speedPid.maxOut = 25000.0f;
    // 位置环PID参数
    YawMotor->positionPid.kp = 100.0f;
    YawMotor->positionPid.ki = 0.0f;
    YawMotor->positionPid.kd = 0.0f;
    YawMotor->positionPid.maxIOut = 3.0f;
    YawMotor->positionPid.maxOut = 25000.0f;

    // 云台Pitch轴电机
    PitchMotor->controlMode = GM6020::POS_MODE;
    // 速度环PID参数
    PitchMotor->speedPid.kp = 200.0f;
    PitchMotor->speedPid.ki = 0.0f;
    PitchMotor->speedPid.kd = 0.0f;
    PitchMotor->speedPid.maxIOut = 3.0f;
    PitchMotor->speedPid.maxOut = 25000.0f;
    // 位置环PID参数
    PitchMotor->positionPid.kp = 100.0f;
    PitchMotor->positionPid.ki = 0.0f;
    PitchMotor->positionPid.kd = 0.0f;
    PitchMotor->positionPid.maxIOut = 3.0f;
    PitchMotor->positionPid.maxOut = 25000.0f;

    YawSet = YawMotor->motorFeedback.positionFdb;
    PitchSet = PitchMotor->motorFeedback.positionFdb;

    // 限幅处理，需要根据实际情况修改
    //  YawMotor->positionSet = Math::FloatConstrain(YawMotor->positionSet, -Math::Pi, Math::Pi);
    // PitchMotor->positionSet = Math::FloatConstrain(PitchMotor->positionSet, PitchLowerLimit, PitchUpperLimit);
}

void GimbalStateRemoteControl::enter()
{
    // 接受遥控器输入
    YawSet -= Dr16::instance()->rc_right_x * 0.008f;
    PitchSet -= Dr16::instance()->rc_right_y * 0.005f;
    YawSet = Math::LoopFloatConstrain(YawSet, 0, Math::PiX2);
    // PitchSet = Math::FloatConstrain(PitchSet, PitchLowerLimit, PitchUpperLimit);
    // PitchSet = Math::LoopFloatConstrain(PitchSet, 0, Math::PiX2);
    // if (PitchSet < PitchUpperLimit && PitchSet > PitchLowerLimit)
    // {
    //     if (abs(PitchSet - PitchUpperLimit) > abs(PitchSet - PitchLowerLimit))
    //     {
    //         PitchSet = PitchUpperLimit;
    //     }
    //     else
    //     {
    //         PitchSet = PitchLowerLimit;
    //     }
    // }
}

void GimbalStateRemoteControl::execute()
{
    // 映射处理
    YawMotor->positionSet = YawSet;
    PitchMotor->positionSet = PitchSet;

    // 外环控制，位置环控制
    YawMotor->positionPid.ref = YawMotor->positionSet;
    YawMotor->positionPid.fdb = YawMotor->motorFeedback.positionFdb;
    if (YawMotor->positionPid.ref - YawMotor->positionPid.fdb >= Math::Pi)
    {
        YawMotor->positionPid.ref -= Math::PiX2;
    }
    else if (YawMotor->positionPid.ref - YawMotor->positionPid.fdb <= -Math::Pi)
    {
        YawMotor->positionPid.ref += Math::PiX2;
    }
    YawMotor->positionPid.UpdateResult();

    // 内环控制，速度环控制
    YawMotor->speedPid.ref = YawMotor->positionPid.result;
    YawMotor->speedPid.fdb = YawMotor->motorFeedback.speedFdb;
    YawMotor->speedPid.UpdateResult();

    YawMotor->currentSet = YawMotor->speedPid.result; // 根据速度PID结果设置电流

    // 外环控制，位置环控制
    PitchMotor->positionPid.ref = PitchMotor->positionSet;
    PitchMotor->positionPid.fdb = PitchMotor->motorFeedback.positionFdb;
    if (PitchMotor->positionPid.ref - PitchMotor->positionPid.fdb >= Math::Pi)
    {
        PitchMotor->positionPid.ref -= Math::PiX2;
    }
    else if (PitchMotor->positionPid.ref - PitchMotor->positionPid.fdb <= -Math::Pi)
    {
        PitchMotor->positionPid.ref += Math::PiX2;
    }

    PitchMotor->positionPid.UpdateResult();

    // 内环控制，速度环控制
    PitchMotor->speedPid.ref = PitchMotor->positionPid.result;
    PitchMotor->speedPid.fdb = PitchMotor->motorFeedback.speedFdb;
    PitchMotor->speedPid.UpdateResult();

    PitchMotor->currentSet = PitchMotor->speedPid.result; // 根据速度PID结果设置电流
}

void GimbalStateRemoteControl::exit()
{
}

void GimbalStateRemoteControl::run()
{
    enter();
    execute();
    exit();
}
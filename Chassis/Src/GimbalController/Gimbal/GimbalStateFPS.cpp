#include "GimbalStateFPS.hpp"
#include "Chassis/ChassisController.hpp"

const float DegreeToRad = Math::Pi / 180.0f;

void GimbalStateFPS::init()
{
    // 初始化云台电机位置
    YawSet = AHRS::instance()->INS.YawTotalAngle * DegreeToRad; // 将初始位置设置为位姿解算的的Yaw
    PitchSet = AHRS::instance()->INS.Pitch * DegreeToRad;       //< Pitch轴电机初始化位置为当前位置

    PitchLowerLimit = -20.2172871 * DegreeToRad; // Pitch轴电机最小角度
    PitchUpperLimit = 21.4231853 * DegreeToRad;  // Pitch轴电机最大角度

    /*-----------------------------------------云台Yaw轴电机-----------------------------------------*/
    YawMotor->controlMode = GM6020::SPD_MODE; // 单环控制？
    // 速度环PID参数
    YawMotor->speedPid.kp = 3300.0f;
    YawMotor->speedPid.ki = 0.0f;
    YawMotor->speedPid.kd = 0.0f;
    YawMotor->speedPid.maxIOut = 3000.0f;
    YawMotor->speedPid.maxOut = 25000.0f;

    // 位置环PID参数
    YawMotor->positionPid.kp = 8.0f;
    YawMotor->positionPid.ki = 0.0f;
    YawMotor->positionPid.kd = 10.0f;
    YawMotor->positionPid.maxIOut = 3.0f;
    YawMotor->positionPid.maxOut = 25000.0f;

    /*----------------------------------------云台Pitch轴电机----------------------------------------*/
    PitchMotor->controlMode = GM6020::SPD_MODE;
    // 速度环PID参数
    PitchMotor->speedPid.kp = 2300.0f;
    PitchMotor->speedPid.ki = 10.0f;
    PitchMotor->speedPid.kd = 5.0f;
    PitchMotor->speedPid.maxIOut = 3000.0f;
    PitchMotor->speedPid.maxOut = 25000.0f;

    // 位置环PID参数
    PitchMotor->positionPid.kp = 20.0f;
    PitchMotor->positionPid.ki = 0.0f;
    PitchMotor->positionPid.kd = 0.0f;
    PitchMotor->positionPid.maxIOut = 3.0f;
    PitchMotor->positionPid.maxOut = 25000.0f;

    // 一阶滤波参数设置
    YawSetFilter.SetTau(0.08f);      // 设置滤波时间常数
    YawSetFilter.SetUpdatePeriod(1); // 设置更新周期

    YawFdbFilter.SetTau(0.08f);      // 设置滤波时间常数
    YawFdbFilter.SetUpdatePeriod(1); // 设置更新周期

    PitchSetFilter.SetTau(0.05f);      //  设置滤波时间常数
    PitchSetFilter.SetUpdatePeriod(1); // 设置更新周期
}

void GimbalStateFPS::enter()
{
    // 接受遥控器输入
    YawSet -= Dr16::instance()->rc_right_x * 0.008f; // 不需要进行限幅，因为使用了totalAngle
    PitchSet += Dr16::instance()->rc_right_y * 0.005f;
    PitchSet = Math::FloatConstrain(PitchSet, PitchLowerLimit, PitchUpperLimit); // 将Pitch轴电机的角度范围限幅到[PitchLowerLimit, PitchUpperLimit]

    // 根据底盘状态设置Yaw轴电机的前馈
    if (ChassisController::instance()->CurrentSate == ChassisController::STATE_FPS)
    {
        ForwardVw = ChassisController::instance()->chassisStateFPS.Vw;
    }
    else if (ChassisController::instance()->CurrentSate == ChassisController::STATE_ROTATE)
    {
        ForwardVw = ChassisController::instance()->chassisStateRotate.Vw;
    }
    else
    {
        ForwardVw = 0.0f;
    }
}

void GimbalStateFPS::execute()
{
    // Yaw一阶滤波
    YawSetFilter.SetInput(YawSet); // 设置YawSetFilter的输入为YawSet
    YawSetFilter.Update();         // 更新YawSetFilter

    YawFdbFilter.SetInput(AHRS::instance()->INS.YawTotalAngle * DegreeToRad); // 设置YawFdbFilter的输入为位姿解算的Yaw
    YawFdbFilter.Update();                                                    // 更新YawFdbFilter

    // Yaw 位置环PID计算
    YawMotor->positionPid.ref = YawSetFilter.GetResult();
    YawMotor->positionPid.fdb = YawFdbFilter.GetResult();
    YawMotor->positionPid.UpdateResult();

    // 设置电机输出
    YawMotor->speedSet = YawMotor->positionPid.result - (3.0f * ForwardVw); // 根据速度PID结果设置电流 + 前馈控制
    YawMotor->setOutput();                                                  // 设置电机输出

    // Pirtch一阶滤波
    PitchSetFilter.SetInput(PitchSet);
    PitchSetFilter.Update();
    // Pitch PID计算
    PitchMotor->positionPid.ref = PitchSetFilter.GetResult();
    PitchMotor->positionPid.fdb = AHRS::instance()->INS.Pitch * DegreeToRad;
    PitchMotor->positionPid.UpdateResult();

    // 设置电机输出
    PitchMotor->speedSet = -PitchMotor->positionPid.result;
    PitchMotor->setOutput();
}

void GimbalStateFPS::exit()
{
}

void GimbalStateFPS::run()
{
    enter();
    execute();
    exit();
}

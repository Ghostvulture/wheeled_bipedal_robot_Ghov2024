#include "GimbalStateRelax.hpp"

void GimbalStateRelax::init()
{
    // 初始化放松状态
    YawMotor->controlMode = GM6020::RELAX_MODE;
    YawMotor->speedPid.Clear();
    YawMotor->positionPid.Clear();

    PitchMotor->controlMode = GM6020::RELAX_MODE;
    PitchMotor->speedPid.Clear();
    PitchMotor->positionPid.Clear();
}

void GimbalStateRelax::enter()
{
    // 初始化放松状态
    YawMotor->controlMode = GM6020::RELAX_MODE;
    PitchMotor->controlMode = GM6020::RELAX_MODE;
}

void GimbalStateRelax::execute()
{
    YawMotor->setOutput();
    PitchMotor->setOutput();
}

void GimbalStateRelax::exit()
{
    // 退出放松状态
}

void GimbalStateRelax::run()
{
    // 运行放松状态
    enter();
    execute();
    exit();
}
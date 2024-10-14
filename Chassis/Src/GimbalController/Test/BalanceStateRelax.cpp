#include "BalanceStateRelax.hpp"

void BalanceStateRelax::init()
{
    // 初始化放松状态
    LMotor->controlMode = LK9025::RELAX_MODE;
    LMotor->speedPid.Clear();
    LMotor->positionPid.Clear();
    LMotor->anglePid.Clear();
    LMotor->omegaPid.Clear();

    RMotor->controlMode = LK9025::RELAX_MODE;
    RMotor->speedPid.Clear();
    RMotor->positionPid.Clear();
    RMotor->anglePid.Clear();
    RMotor->omegaPid.Clear();

}

void BalanceStateRelax::enter()
{
    // 初始化放松状态
    LMotor->controlMode = LK9025::RELAX_MODE;
    RMotor->controlMode = LK9025::RELAX_MODE;
}

void BalanceStateRelax::execute()
{
    LMotor->setOutput();
    RMotor->setOutput();
}

void BalanceStateRelax::exit()
{
    // 退出放松状态
}

void BalanceStateRelax::run()
{
    // 运行放松状态
    enter();
    execute();
    exit();
}
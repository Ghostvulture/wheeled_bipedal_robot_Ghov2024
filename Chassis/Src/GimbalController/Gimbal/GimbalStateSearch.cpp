#include "GimbalStateSearch.hpp"

void GimbalStateSearch::init()
{
    // 暂时按照放松状态初始化
    // 初始化放松状态
    YawMotor->controlMode = GM6020::SPD_MODE;
    YawMotor->speedPid.Clear();
    YawMotor->positionPid.Clear();

    PitchMotor->controlMode = GM6020::POS_MODE;
    PitchMotor->speedPid.Clear();
    PitchMotor->positionPid.Clear();
}

void GimbalStateSearch::enter()
{
}

void GimbalStateSearch::execute()
{
    // pitch轴电机位置控制，可以直接使用视觉发过来的数据
    PitchMotor->positionSet = PitchSet_Ref; // 可能需要更具实际情况进行数值处理
    PitchMotor->setOutput();
    // yaw轴电机位置控制，需要配合小陀螺模式
    YawMotor->speedPid.ref = YawSet_Ref; // 可能需要更具实际情况进行数值处理
    YawMotor->speedPid.fdb = AHRS::instance()->INS.Yaw;
    YawMotor->speedPid.UpdateResult();
    YawMotor->currentSet = YawMotor->speedPid.result;
}

void GimbalStateSearch::exit()
{
    // 退出放松状态
}

void GimbalStateSearch::run()
{
    // 运行放松状态
    enter();
    execute();
    exit();
}

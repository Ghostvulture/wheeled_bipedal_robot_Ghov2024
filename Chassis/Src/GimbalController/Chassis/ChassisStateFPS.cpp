#include "ChassisStateFPS.hpp"
#include "GimbalController.hpp"

float debug_chassis_follow = 0.0f;

/**
 * @brief 初始化函数，设置各种数据
 */
void ChassisStateFPS::init()
{
    Vx = 0.0f;
    Vy = 0.0f;
    Vw = 0.0f;

    // 设置PID参数
    ChassisYawSpeedPid.kp = 5.0f;
    ChassisYawSpeedPid.ki = 0.0f;
    ChassisYawSpeedPid.kd = 10.0f;
    ChassisYawSpeedPid.maxIOut = 0.0f;
    ChassisYawSpeedPid.maxOut = 15.0f;

    // 设置滤波器参数
    VxFilter.SetTau(0.16f);
    VxFilter.SetUpdatePeriod(5);

    VyFilter.SetTau(0.12f);
    VyFilter.SetUpdatePeriod(5);

    VwFilter.SetTau(0.04f);
    VwFilter.SetUpdatePeriod(5);

    // 发送数据给底盘
    uint8_t xyData[8];
    uint8_t wData[8];
    BoardConnectivity::instance()->BoardConnectivity_Float2Byte(Vx, xyData);
    BoardConnectivity::instance()->BoardConnectivity_Float2Byte(Vy, xyData + 4);

    BoardConnectivity::instance()->BoardConnectivity_Add2Memory(xyData, 0xB1, 8, BoardConnectivity::BOARD_CONNECTIVITY_CAN_2, BoardConnectivity::BOARD_CONNECTIVITY_SEND);

    BoardConnectivity::instance()->BoardConnectivity_Float2Byte(Vw, wData);
    BoardConnectivity::instance()->BoardConnectivity_Float2Byte(Vw, wData + 4);
    BoardConnectivity::instance()->BoardConnectivity_Add2Memory(wData, 0xB2, 8, BoardConnectivity::BOARD_CONNECTIVITY_CAN_2, BoardConnectivity::BOARD_CONNECTIVITY_SEND);
}

void ChassisStateFPS::enter()
{
    // 将遥控器的数据转化为底盘速度，在这里乘以一个常数，是为了调整速度的大小。单位为m/s。
    Vx = Dr16::instance()->rc_left_y * 5;
    Vy = Dr16::instance()->rc_left_x * 5;

    // 对速度进行滤波
    VxFilter.SetInput(Vx);
    VxFilter.Update();
    Vx = VxFilter.GetResult();

    VyFilter.SetInput(Vy);
    VyFilter.Update();
    Vy = VyFilter.GetResult();

    relativeAngle = GimbalController::instance()->YawMotor.motorFeedback.positionFdb - GimbalController::instance()->YawMotor.Offset;
    // 防止角度突变
    if (relativeAngle >= Math::Pi)
    {
        relativeAngle -= Math::PiX2;
    }
    else if (relativeAngle <= -Math::Pi)
    {
        relativeAngle += Math::PiX2;
    }

    // 底盘跟随云台PID计算
    ChassisYawSpeedPid.ref = relativeAngle;
    ChassisYawSpeedPid.fdb = 0;
    ChassisYawSpeedPid.UpdateResult();
    Vw = ChassisYawSpeedPid.result;

    // 对旋转速度进行滤波
    VwFilter.SetInput(Vw);
    VwFilter.Update();
    VwFilter.GetResult();
    Vw = VwFilter.GetResult();
}

void ChassisStateFPS::execute()
{
    // 发送数据给底盘
    uint8_t xyData[8];
    uint8_t wData[8];

    BoardConnectivity::instance()->BoardConnectivity_Float2Byte(Vx, xyData);
    BoardConnectivity::instance()->BoardConnectivity_Float2Byte(Vy, xyData + 4);
    BoardConnectivity::instance()->BoardConnectivity_Add2Memory(xyData, 0xB1, 8, BoardConnectivity::BOARD_CONNECTIVITY_CAN_2, BoardConnectivity::BOARD_CONNECTIVITY_SEND);

    BoardConnectivity::instance()->BoardConnectivity_Float2Byte(Vw, wData);
    BoardConnectivity::instance()->BoardConnectivity_Float2Byte(Vw, wData + 4);
    BoardConnectivity::instance()->BoardConnectivity_Add2Memory(wData, 0xB2, 8, BoardConnectivity::BOARD_CONNECTIVITY_CAN_2, BoardConnectivity::BOARD_CONNECTIVITY_SEND);
}

void ChassisStateFPS::exit()
{
}

void ChassisStateFPS::run()
{
    enter();
    execute();
    exit();
}

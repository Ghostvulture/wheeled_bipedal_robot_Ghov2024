#include "GimbalController.hpp"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void GimbalController::init()
{
    YawMotor.controlMode = GM6020::RELAX_MODE;
    YawMotor.Offset = 4.56359291 - Math::Pi;
    YawMotor.setOutput();
    GMMotorHandler::instance()->registerMotor(&YawMotor, &hcan1, 0x205); // 挂载云台电机

    PitchMotor.controlMode = GM6020::RELAX_MODE;
    PitchMotor.Offset = 0.0f;
    PitchMotor.setOutput();
    GMMotorHandler::instance()->registerMotor(&PitchMotor, &hcan1, 0x206); // 挂载云台电机
}

void GimbalController::run()
{
    if (Dr16::instance()->left_sw == Dr16::RC_SW_DOWN) // 如果遥控器左边的拨杆向下
    {
        if (CurrentSate != STATE_RELAX)
        {
            RelaxState.init(); // 放松状态
            CurrentSate = STATE_RELAX;
        }
        RelaxState.run(); // 放松状态
    }
    else if (Dr16::instance()->left_sw == Dr16::RC_SW_MID) // 如果遥控器左边的拨杆中间
    {
        if (CurrentSate != STATE_FPS)
        {
            FPSState.init();
            CurrentSate = STATE_FPS;
        }
        FPSState.run(); // 底盘跟随云台状态
    }
    else if (Dr16::instance()->left_sw == Dr16::RC_SW_UP) // 如果遥控器左边的拨杆向上
    {
        if (CurrentSate != STATE_REMOTE_CONTROL)
        {
            RemoteControlState.init();
            CurrentSate = STATE_REMOTE_CONTROL;
        }
        RemoteControlState.run();
    }
    else // 默认状态
    {
        if (CurrentSate != STATE_RELAX)
        {
            RelaxState.init(); // 放松状态
            CurrentSate = STATE_RELAX;
        }
        RelaxState.run();
    }
}

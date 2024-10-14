#include "BalanceController.hpp"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void BalanceController::init()
{
    RMotor.controlMode = LK9025::RELAX_MODE;
    RMotor.setOutput();
    LKMotorHandler::instance()->registerMotor(&RMotor, &hcan1, 0x141); 

    LMotor.controlMode = LK9025::RELAX_MODE;
    LMotor.setOutput();
    LKMotorHandler::instance()->registerMotor(&LMotor, &hcan1, 0x142); 

}

void BalanceController::run()
{
    if (Dr16::instance()->left_sw == Dr16::RC_SW_DOWN) // 如果遥控器左边的拨杆向下
    {
        if (CurrentSate != STATE_RELAX)
            {
                RelaxState.init(); // 放松状态
                CurrentSate = STATE_RELAX;
            }
            RelaxState.run(); // 放松状态
        // if (CurrentSate != STATE_REMOTE_CONTROL)
        // {
        //     RemoteControlState.init();
        //     CurrentSate = STATE_REMOTE_CONTROL;
        // }
        // RemoteControlState.run(); 
    }
    else if (Dr16::instance()->left_sw == Dr16::RC_SW_MID) // 如果遥控器左边的拨杆中间
    {

        if (CurrentSate != STATE_RELAX)
        {
            RelaxState.init(); // 放松状态
            CurrentSate = STATE_RELAX;
        }
        RelaxState.run(); // 放松状态
    }
    else if (Dr16::instance()->left_sw == Dr16::RC_SW_UP) // 如果遥控器左边的拨杆向上
    {
        if (CurrentSate != STATE_STAND_CONTROL)
        {
            StandState.init();
            CurrentSate = STATE_STAND_CONTROL;
        }
        StandState.run();
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

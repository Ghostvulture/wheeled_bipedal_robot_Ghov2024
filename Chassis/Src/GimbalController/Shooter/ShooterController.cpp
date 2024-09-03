#include "ShooterController.hpp"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void ShooterController::init()
{
    /*--------------------左摩擦轮--------------------*/
    LeftFricMotor.controlMode = GM3508::SPD_MODE;

    // 速度环PID参数
    LeftFricMotor.speedPid.kp = 100.0f;
    LeftFricMotor.speedPid.ki = 0.0f;
    LeftFricMotor.speedPid.kd = 0.0f;

    LeftFricMotor.speedSet = 0;
    LeftFricMotor.setOutput();

    /*--------------------右摩擦轮--------------------*/
    RightFricMotor.controlMode = GM3508::SPD_MODE;

    // 速度环PID参数
    RightFricMotor.speedPid.kp = 100.0f;
    RightFricMotor.speedPid.ki = 0.0f;
    RightFricMotor.speedPid.kd = 0.0f;

    RightFricMotor.speedSet = 0;
    RightFricMotor.setOutput();

    /*--------------------拨弹轮--------------------*/
    TriggerMotor.controlMode = GM2006::SPD_MODE;

    // 速度环PID参数
    TriggerMotor.speedPid.kp = 100.0f;
    TriggerMotor.speedPid.ki = 0.0f;
    TriggerMotor.speedPid.kd = 0.0f;

    TriggerMotor.speedSet = 0;
    TriggerMotor.setOutput();

    // 注册电机
    GMMotorHandler::instance()->registerMotor(&LeftFricMotor, &hcan1, 0x201);
    GMMotorHandler::instance()->registerMotor(&RightFricMotor, &hcan1, 0x202);
    GMMotorHandler::instance()->registerMotor(&TriggerMotor, &hcan1, 0x203);
}

void ShooterController::run()
{
    if (Dr16::instance()->right_sw == Dr16::RC_SW_DOWN) // 如果遥控器左边的拨杆向下
    {
        if (CurrentSate != STATE_RELAX)
        {
            CurrentSate = STATE_RELAX;
        }

        LeftFricMotor.speedSet = 0;
        RightFricMotor.speedSet = 0;
        TriggerMotor.speedSet = 0;

        LeftFricMotor.setOutput();
        RightFricMotor.setOutput();
        TriggerMotor.setOutput();
    }
    else if (Dr16::instance()->right_sw == Dr16::RC_SW_MID) // 如果遥控器左边的拨杆中间
    {
        if (CurrentSate != STATE_PREHEAT)
        {
            CurrentSate = STATE_PREHEAT;
        }

        LeftFricMotor.speedSet = 25;
        RightFricMotor.speedSet = 25;
        TriggerMotor.speedSet = 0;

        LeftFricMotor.setOutput();
        RightFricMotor.setOutput();
        TriggerMotor.setOutput();
    }
    else if (Dr16::instance()->right_sw == Dr16::RC_SW_UP) // 如果遥控器左边的拨杆向上
    {
        if (CurrentSate != STATE_FIRE)
        {
            CurrentSate = STATE_FIRE;
        }

        LeftFricMotor.speedSet = 35;
        RightFricMotor.speedSet = 35;
        TriggerMotor.speedSet = -24 * 19;

        LeftFricMotor.setOutput();
        RightFricMotor.setOutput();
        TriggerMotor.setOutput();
    }
    else // 默认状态
    {
        if (CurrentSate != STATE_RELAX)
        {
            CurrentSate = STATE_RELAX;
        }

        LeftFricMotor.speedSet = 0;
        RightFricMotor.speedSet = 0;
        TriggerMotor.speedSet = 0;

        LeftFricMotor.setOutput();
        RightFricMotor.setOutput();
        TriggerMotor.setOutput();
    }
}

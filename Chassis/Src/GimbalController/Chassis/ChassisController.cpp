#include "ChassisController.hpp"

void ChassisController::init()
{
    chassisStateRelax.init(); // 初始化为放松状态
}

void ChassisController::run()
{
    if (Dr16::instance()->left_sw == Dr16::RC_SW_DOWN) // 如果遥控器左边的拨杆向下
    {
        // 如果初次进入这个状态，初始化
        if (CurrentSate != STATE_RELAX)
        {
            chassisStateRelax.init(); // 放松状态
            CurrentSate = STATE_RELAX;
        }
        chassisStateRelax.run();
    }
    else if (Dr16::instance()->left_sw == Dr16::RC_SW_MID) // 如果遥控器左边的拨杆中间
    {
        if (CurrentSate != STATE_FPS)
        {
            chassisStateFPS.init();
            CurrentSate = STATE_FPS;
        }
        chassisStateFPS.run();
    }
    else if (Dr16::instance()->left_sw == Dr16::RC_SW_UP) // 如果遥控器左边的拨杆向上
    {
        if (CurrentSate != STATE_ROTATE)
        {
            chassisStateRotate.init();
            CurrentSate = STATE_ROTATE;
        }
        chassisStateRotate.run();
    }
    else
    {
        // 如果初次进入这个状态，初始化
        if (CurrentSate != STATE_RELAX)
        {
            chassisStateRelax.init(); // 放松状态
            CurrentSate = STATE_RELAX;
        }
        chassisStateRelax.run();
    }
}

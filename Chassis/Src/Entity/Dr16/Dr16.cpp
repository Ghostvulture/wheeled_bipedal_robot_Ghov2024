#include "Dr16.hpp"

float Dr16::mapAvix(int16_t ch)
{
    return (float)ch / (float)RC_CH_OFFSET_MAX;
}

Dr16::Dr16()
{
    rc_raw = get_remote_control_raw();

    rc_right_x = mapAvix(rc_raw->rc.ch[0]);
    rc_right_y = mapAvix(rc_raw->rc.ch[1]);
    rc_left_x = mapAvix(rc_raw->rc.ch[2]);
    rc_left_y = mapAvix(rc_raw->rc.ch[3]);

    right_prev_sw = (RC_SWITCH_STATE_TYPE)rc_raw->rc.s[0];
    right_sw = (RC_SWITCH_STATE_TYPE)rc_raw->rc.s[0];
    left_prev_sw = (RC_SWITCH_STATE_TYPE)rc_raw->rc.s[1];
    left_sw = (RC_SWITCH_STATE_TYPE)rc_raw->rc.s[1];
}

Dr16::~Dr16()
{
}

Dr16::PC_KEY_STATE_TYPE Dr16::getKeyStatus(PC_KEY_TYPE key)
{
    uint16_t mask = (uint16_t)key;
    uint16_t current = key_status.current_state & mask;
    uint16_t previous = key_status.previous_state & mask;

    if (current && previous)
    {
        return PC_KEY_DOWN;
    }
    else if (!current && !previous)
    {
        return PC_KEY_UP;
    }
    else if (previous && !current)
    {
        return PC_KEY_FALL;
    }
    else
    {
        return PC_KEY_RISE;
    }
}

void Dr16::updateRcStatus()
{
    // 映射遥控器摇杆的通道数据到[-1, 1]的范围
    rc_right_x = mapAvix(rc_raw->rc.ch[0]);
    rc_right_y = mapAvix(rc_raw->rc.ch[1]);
    rc_left_x = mapAvix(rc_raw->rc.ch[2]);
    rc_left_y = mapAvix(rc_raw->rc.ch[3]);

    // 更新遥控器拨杆状态
    if (left_sw != rc_raw->rc.s[1])
        left_prev_sw = left_sw;
    if (right_sw != rc_raw->rc.s[0])
        right_prev_sw = right_sw;

    left_sw = (RC_SWITCH_STATE_TYPE)rc_raw->rc.s[1];
    right_sw = (RC_SWITCH_STATE_TYPE)rc_raw->rc.s[0];

    if (left_prev_sw == RC_SW_UP && left_sw == RC_SW_MID)
        left_sw_change = RC_SWITCH_U2M;
    else if (left_prev_sw == RC_SW_MID && left_sw == RC_SW_UP)
        left_sw_change = RC_SWITCH_M2U;
    else if (left_prev_sw == RC_SW_MID && left_sw == RC_SW_DOWN)
        left_sw_change = RC_SWITCH_M2D;
    else if (left_prev_sw == RC_SW_DOWN && left_sw == RC_SW_MID)
        left_sw_change = RC_SWITCH_D2M;

    if (right_prev_sw == RC_SW_UP && right_sw == RC_SW_MID)
        right_sw_change = RC_SWITCH_U2M;
    else if (right_prev_sw == RC_SW_MID && right_sw == RC_SW_UP)
        right_sw_change = RC_SWITCH_M2U;
    else if (right_prev_sw == RC_SW_MID && right_sw == RC_SW_DOWN)
        right_sw_change = RC_SWITCH_M2D;
    else if (right_prev_sw == RC_SW_DOWN && right_sw == RC_SW_MID)
        right_sw_change = RC_SWITCH_D2M;
}

void Dr16::updateKeyStatus()
{
    // 当前按键状态与上一次按键状态不同时更新
    if (key_status.current_state != rc_raw->key.v)
    {
        key_status.previous_state = key_status.current_state;
        key_status.current_state = rc_raw->key.v;
    }
}

void Dr16::updateData()
{
    updateRcStatus();
    updateKeyStatus();
}

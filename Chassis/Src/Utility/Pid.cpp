#include "Pid.hpp"

Pid::Pid() : mode(PID_POSITION),
             kp(0.0f),
             ki(0.0f),
             kd(0.0f),
             maxOut(0.0f),
             maxIOut(0.0f)
{
    dBuf[0] = dBuf[1] = dBuf[2] = 0.0f;
    err[0] = err[1] = err[2] = 0.0f;
}

void Pid::Init(void)
{
    Clear();
}

void Pid::UpdateResult(void)
{
    err[2] = err[1];
    err[1] = err[0];

    if (mode == PID_POSITION)
    {
        err[0] = ref - fdb;

        pResult = kp * err[0];

        iResult += ki * err[0];
        iResult = Math::LimitMax(iResult, maxIOut);

        dBuf[2] = dBuf[1];
        dBuf[1] = dBuf[0];
        dBuf[0] = err[0] - err[1];
        dResult = kd * dBuf[0];
    }
    else if (mode == PID_DELTA)
    {
        pResult = kp * (err[0] - err[1]);
        iResult = ki * err[0];
        dBuf[2] = dBuf[1];
        dBuf[1] = dBuf[0];
        dBuf[0] = (err[0] - 2.0f * err[1] + err[2]);
        dResult = kd * dBuf[0];
    }

    result = pResult + iResult + dResult;
    result = Math::LimitMax(result, maxOut);
}

void Pid::Clear(void)
{
    dBuf[0] = dBuf[1] = dBuf[2] = 0.0f;
    err[0] = err[1] = err[2] = 0.0f;
    pResult = iResult = dResult = result = 0.0f;
    ref = fdb = 0.0f;
    iResult = 0.0f;
    pResult = 0.0f;
    dResult = 0.0f;
}

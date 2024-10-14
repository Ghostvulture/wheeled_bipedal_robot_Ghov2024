#ifndef PID_HPP
#define PID_HPP

#include "Math.hpp"


/**
 * @brief Proportion integration differentiation control algorithm
 */
#ifdef __cplusplus

class Pid
{
public:
    /**
     * @brief PID mode type enumeration.
     */
    enum PidModeType
    {
        /** Position mode, basic type of PID */
        PID_POSITION = 0,

        /** Delta mode, differentiation of position mode, not used*/
        PID_DELTA
    };

    /**
     * @brief PID mode.
     */
    PidModeType mode;

    /**
     * @brief coefficient of proportion.
     */
    float kp;

    /**
     * @brief coefficient of integration.
     */
    float ki;

    /**
     * @brief coefficient of differentiation.
     */
    float kd;

    /**
     * @brief set value or target value of control.
     */
    float ref;

    /**
     * @brief real value or feedback value of control from sensor.
     */
    float fdb;

    /**
     * @brief result of proportion.
     */
    float pResult;

    /**
     * @brief result of integration.
     */
    float iResult;

    /**
     * @brief result of differentiation.
     */
    float dResult;

    /**
     * @brief output result of PID calculation, pResult + iResult + dResult at range [-maxOut, maxOut]
     */
    float result;

    /**
     * @brief absolute value of max output result.
     */
    float maxOut;

    /**
     * @brief absolute value of max output result of integration.
     */
    float maxIOut;

    /**
     * @brief intermediate variable for differentiation.
     */
    float dBuf[3];

    /**
     * @brief intermediate variable for differentiation.
     */
    float err[3];

    /**
     * @brief Construct Pid, set all member variables to zero
     */
    Pid();

    /**
     * @brief Initialization Pid, set all intermediate variables and result to zero
     */
    void Init();

    /**
     * @brief Update pResult, iResult and dResult, after limit max value, update result
     */
    void UpdateResult();

    /**
     * @brief Set all intermediate variables and result to zero
     */
    void Clear();

    /**
     * @brief Update iResult
     * @param _err error of control, target - feedback
     * @note called in GM6020::ControlUpdate() for unknown use
     */
    void UpdateIResult(float _err)
    {
        iResult += ki * _err;
        iResult = Math::LimitMax(iResult, maxIOut);
    }
};

#endif
#endif

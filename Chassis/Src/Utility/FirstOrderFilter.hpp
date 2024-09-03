#ifndef MATH_FIRST_ORDER_FILTER_H_
#define MATH_FIRST_ORDER_FILTER_H_

#include <stdint.h>
#include <math.h>
#include "arm_math.h"

class FirstOrderFilter
{
private:
    float Input;        // 设置滤波器的输入值
    float OutPut;       // 设置滤波器的输出值
    float Tau;          // 设置滤波器的时间常数
    float UpdatePeriod; // 设置滤波器的更新周期，单位为秒，但是设置时以毫秒为单位

public:
    /**
     * @brief 设置滤波器的输入值
     */
    void SetInput(float in)
    {
        Input = in;
    }
    /*
     * @brief 设置滤波器的时间常数
     */
    void SetTau(float tau)
    {
        Tau = tau;
    }

    /**
     * @brief 设置滤波器的输出值
     */
    void SetResult(float out)
    {
        OutPut = out;
    }

    /**
     * @brief 设置滤波器的更新周期
     */
    void SetUpdatePeriod(float t)
    {
        UpdatePeriod = t * 0.001f;
    } /* t in ms */

    /**
     * @brief 获取滤波器的输出值
     */
    float GetResult()
    {
        return OutPut;
    }

    /**
     * @brief 获取滤波器的输入值
     */
    float GetTau()
    {
        return Tau;
    }

    /**
     * @brief 获取滤波器的更新周期
     */
    float GetUpdatePeriod()
    {
        return UpdatePeriod;
    }

    /**
     * @brief 初始化滤波器
     */
    void Init()
    {
        Clear();
        UpdatePeriod = 0.001f;
        Tau = 0.25f;
    }

    /**
     * @brief 更新滤波器
     */
    void Update()
    {
        float a = UpdatePeriod / (Tau);
        OutPut = (1 - a) * OutPut + a * Input;
    }

    void Clear()
    {
        OutPut = 0;
        Input = 0;
    }
};

#endif /* MATH_FIRST_ORDER_FILTER_H_ */

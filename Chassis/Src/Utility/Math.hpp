#ifndef MATH_HPP
#define MATH_HPP

#include "stm32f4xx.h"
#include "math.h"
#include "arm_math.h"
class Math
{
public:
    static const float Pi;      ///< 圆周率
    static const float PiX2;    ///< 2倍圆周率
    static const float Gravity; ///< 重力加速度

    /**
     * @brief 计算两个角度之间的最短角度。范围是[-π, π]。
     * @param target 目标角度
     * @param current 当前角度
     * @return 两个角度之间的最短角度
     */
    static float calculateShortestAngle(float target, float current)
    {
        float difference = target - current;

        if (difference > Pi)
        {
            difference -= 2 * Pi; // 如果差值超过π，减去2π
        }
        else if (difference < -Pi)
        {
            difference += 2 * Pi; // 如果差值小于-π，加上2π
        }

        return difference;
    }

    /**
     * @brief 将输入按照最大值和最小值形成的周期限制
     * 采用模运算，可以将算法的时间复杂度降低到O(1)。
     * @param input 输入值
     * @param minValue 最小值
     * @param maxValue 最大值
     * @return 限制在[minValue, maxValue]之间的值
     */
    static float LoopFloatConstrain(float input, float minValue, float maxValue)
    {
        if (maxValue < minValue)
            return input; // 保持原样，因为输入的范围不合逻辑

        const float range = maxValue - minValue;

        if (range == 0.0f)
            return minValue; // 最大值和最小值相等时直接返回任一值

        float normalizedInput = input - minValue;

        normalizedInput = fmod(normalizedInput, range); // 使用模运算确保结果在[minValue, maxValue]之间

        if (normalizedInput < 0)
            normalizedInput += range; // 确保结果非负

        return minValue + normalizedInput;
    }

    /**
     * @brief 对输入进行限幅在一个明确的范围内。
     * @param Input 输入值
     * @param minValue 最小值
     * @param maxValue 最大值
     * @return 限幅后的值，范围会在[minValue, maxValue]之间
     */
    static float FloatConstrain(float Input, float minValue, float maxValue)
    {
        if (maxValue < minValue)
            return Input;

        if (Input > maxValue)
            Input = maxValue;

        else if (Input < minValue)
            Input = minValue;

        return Input;
    }

    /**
     * @brief 对输入进行限幅。
     * @param _input 输入值
     * @param _max 最大绝对值
     * @return 限幅后的值，绝对值不会超过 _max
     */
    static float LimitMax(float _input, float _max)
    {
        if (_input > _max)
        {
            return _max;
        }
        else if (_input < -_max)
        {
            return -_max;
        }
        return _input;
    }

    /**
     * @brief 将浮点数转换为固定小数点表示。
     * @param _inNum 输入的浮点数。
     * @param _inMin 输入数值的最小可能值。
     * @param _inPrecision 转换的精度，表示固定小数点的步长。
     * @return 返回转换后的固定小数点表示的无符号整数。
     */
    static uint32_t ConvertToFixed(float _inNum, float _inMin, float _inPrecision)
    {
        return (uint32_t)((_inNum - _inMin) / _inPrecision);
    }

    /**
     * @brief 从固定小数点表示转换回浮点数。
     * @param _inNum 固定小数点表示的数值。
     * @param _inMin 转换时使用的最小可能值。
     * @param _inPrecision 转换时使用的精度，即固定小数点的步长。
     * @return 返回转换回的浮点数。
     */
    static float ConvertFromFixed(uint32_t _inNum, float _inMin, float _inPrecision)
    {
        return (float)(_inNum)*_inPrecision + _inMin;
    }

    /**
     * @brief 计算一个数的逆平方根，即计算一个数的平方根的倒数。
     * @param x 输入的浮点数。
     * @return 返回输入数的逆平方根。
     * @note 使用快速逆平方根算法，在1990年代，一个称为“快速逆平方根”的算法因其在《雷神之锤III竞技场》（Quake III Arena）游戏的源代码中被发现而变得广为人知。这个算法使用了一种数学技巧和近似方法，能够非常迅速地计算逆平方根，其精度对于游戏图形来说足够好。
     */
    static float invSqrt(float x)
    {
        float xhalf = 0.5f * x;
        int i = *(int *)&x;             // 获取浮点值的位表示
        i = 0x5f3759df - (i >> 1);      // 初始猜测
        x = *(float *)&i;               // 将位转换回浮点数
        x = x * (1.5f - xhalf * x * x); // 牛顿迭代步骤
        return 1.0f / x;
    }

    /**
     * @brief 计算浮点数的绝对值。
     * @param x 输入的浮点数。
     * @return 返回输入数的绝对值。
     */
    static float abs(float x)
    {
        return x > 0.0f ? x : -x;
    }
};

#endif

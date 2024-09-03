#ifndef _AHRS_HPP
#define _AHRS_HPP

#include "math.h"
#include "BMI088.hpp"
#include "IST8310.hpp"
#include "bsp_dwt.h"
#include "QuaternionEKF.h"
#include "arm_math.h"
#include "user_lib.h"
#include "stdint.h"
#include "string.h"

#define X 0
#define Y 1
#define Z 2

#define INS_TASK_PERIOD 1 // ms

class AHRS
{
public:
    AHRS()
    {
        INS_DWT_Count = 0;
        dt = 0;
        t = 0;
    }
    ~AHRS() {}
    /**
     * @brief 初始化惯导解算系统
     *
     */

    typedef struct
    {
        float Gyro[3];  // 角速度
        float Accel[3]; // 加速度
        // 还需要增加角速度数据
        float Roll;
        float Pitch;
        float Yaw;
        float YawTotalAngle;
    } attitude_t; // 最终解算得到的角度,以及yaw转动的总角度(方便多圈控制)

    typedef struct
    {
        float q[4]; // 四元数估计值

        float MotionAccel_b[3]; // 机体坐标加速度
        float MotionAccel_n[3]; // 绝对系加速度

        float AccelLPF; // 加速度低通滤波系数

        // bodyframe在绝对系的向量表示
        float xn[3];
        float yn[3];
        float zn[3];

        // 加速度在机体系和XY两轴的夹角
        // float atanxz;
        // float atanyz;

        // IMU量测值
        float Gyro[3];  // 角速度
        float Accel[3]; // 加速度
        // 位姿
        float Roll;
        float Pitch;
        float Yaw;
        float YawTotalAngle;

        uint8_t init;
    } INS_t;

    /* 用于修正安装误差的参数 */
    typedef struct
    {
        uint8_t flag;

        float scale[3];

        float Yaw;
        float Pitch;
        float Roll;
    } IMU_Param_t;

    attitude_t *INS_Init(void);

    INS_t INS;

    IMU_Param_t IMU_Param;

    const float xb[3] = {1, 0, 0};
    const float yb[3] = {0, 1, 0};
    const float zb[3] = {0, 0, 1};

    // 用于获取两次采样之间的时间间隔
    uint32_t INS_DWT_Count;
    float dt;
    float t;

    void AHRS_Update();

    /**
     * @brief 四元数更新函数,即实现dq/dt=0.5Ωq
     *
     * @param q  四元数
     * @param gx
     * @param gy
     * @param gz
     * @param dt 距离上次调用的时间间隔
     */
    void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt);

    /**
     * @brief 四元数转换成欧拉角 ZYX
     *
     * @param q
     * @param Yaw
     * @param Pitch
     * @param Roll
     */
    void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll);

    /**
     * @brief ZYX欧拉角转换为四元数
     *
     * @param Yaw
     * @param Pitch
     * @param Roll
     * @param q
     */
    void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q);

    /**
     * @brief 机体系到惯性系的变换函数
     *
     * @param vecBF body frame
     * @param vecEF earth frame
     * @param q
     */
    void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);

    /**
     * @brief 惯性系转换到机体系
     *
     * @param vecEF
     * @param vecBF
     * @param q
     */
    void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);

    /**
     * @brief reserved.用于修正IMU安装误差与标度因数误差,即陀螺仪轴和云台轴的安装偏移
     *
     *
     * @param param IMU参数
     * @param gyro  角速度
     * @param accel 加速度
     */
    static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3])
    {
        static float lastYawOffset, lastPitchOffset, lastRollOffset;
        static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;
        float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;

        if (fabsf(param->Yaw - lastYawOffset) > 0.001f ||
            fabsf(param->Pitch - lastPitchOffset) > 0.001f ||
            fabsf(param->Roll - lastRollOffset) > 0.001f || param->flag)
        {
            cosYaw = arm_cos_f32(param->Yaw / 57.295779513f);
            cosPitch = arm_cos_f32(param->Pitch / 57.295779513f);
            cosRoll = arm_cos_f32(param->Roll / 57.295779513f);
            sinYaw = arm_sin_f32(param->Yaw / 57.295779513f);
            sinPitch = arm_sin_f32(param->Pitch / 57.295779513f);
            sinRoll = arm_sin_f32(param->Roll / 57.295779513f);

            // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
            c_11 = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
            c_12 = cosPitch * sinYaw;
            c_13 = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
            c_21 = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
            c_22 = cosYaw * cosPitch;
            c_23 = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
            c_31 = -cosPitch * sinRoll;
            c_32 = sinPitch;
            c_33 = cosPitch * cosRoll;
            param->flag = 0;
        }
        float gyro_temp[3];
        for (uint8_t i = 0; i < 3; ++i)
            gyro_temp[i] = gyro[i] * param->scale[i];

        gyro[X] = c_11 * gyro_temp[X] +
                  c_12 * gyro_temp[Y] +
                  c_13 * gyro_temp[Z];
        gyro[Y] = c_21 * gyro_temp[X] +
                  c_22 * gyro_temp[Y] +
                  c_23 * gyro_temp[Z];
        gyro[Z] = c_31 * gyro_temp[X] +
                  c_32 * gyro_temp[Y] +
                  c_33 * gyro_temp[Z];

        float accel_temp[3];
        for (uint8_t i = 0; i < 3; ++i)
            accel_temp[i] = accel[i];

        accel[X] = c_11 * accel_temp[X] +
                   c_12 * accel_temp[Y] +
                   c_13 * accel_temp[Z];
        accel[Y] = c_21 * accel_temp[X] +
                   c_22 * accel_temp[Y] +
                   c_23 * accel_temp[Z];
        accel[Z] = c_31 * accel_temp[X] +
                   c_32 * accel_temp[Y] +
                   c_33 * accel_temp[Z];

        lastYawOffset = param->Yaw;
        lastPitchOffset = param->Pitch;
        lastRollOffset = param->Roll;
    }

    static void InitQuaternion(float *init_q4)
    {
        float acc_init[3] = {0};
        float gravity_norm[3] = {0, 0, 1}; // 导航系重力加速度矢量,归一化后为(0,0,1)
        float axis_rot[3] = {0};           // 旋转轴
        // 读取100次加速度计数据,取平均值作为初始值
        for (uint8_t i = 0; i < 100; ++i)
        {
            acc_init[X] += BMI088::instance()->bmi088_data.acc_data.x;
            acc_init[Y] += BMI088::instance()->bmi088_data.acc_data.y;
            acc_init[Z] += BMI088::instance()->bmi088_data.acc_data.z;
            DWT_Delay(0.001);
        }
        for (uint8_t i = 0; i < 3; ++i)
            acc_init[i] /= 100;
        Norm3d(acc_init);
        // 计算原始加速度矢量和导航系重力加速度矢量的夹角
        float angle = acosf(Dot3d(acc_init, gravity_norm));
        Cross3d(acc_init, gravity_norm, axis_rot);
        Norm3d(axis_rot);
        init_q4[0] = cosf(angle / 2.0f);
        for (uint8_t i = 0; i < 2; ++i)
            init_q4[i + 1] = axis_rot[i] * sinf(angle / 2.0f); // 轴角公式,第三轴为0(没有z轴分量)
    }

    static AHRS *instance()
    {
        static AHRS ahrs;
        return &ahrs;
    }
};

#endif // !_AHRS_HPP

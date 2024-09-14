
#ifndef ODOMETER_HPP
#define ODOMETER_HPP

#include "main.h"
#include "LKMotor.hpp"
#include "Pid.hpp"

class Odometer
{
private:
    //传入LKMotor的引用
    LKMotor &LMotor;
    LKMotor &RMotor;

    //车体实际常数
    const float WHEEL_RADIUS = 0.1;//m
    const float BODY_RADIUS = 0.3;//m

    //位移refference值
    float DisplacementSet = 0;

    //里程计数据,使用弧度制的pisitionFdb
    float L_last_position = 0;
    float L_delta_position = 0;

    float R_last_position = 0;
    float R_delta_position = 0;

    //解算得里程计数据
    float delta_x = 0;
    float delta_theta = 0;

    //pid
    Pid DisplacementPid;



public:
    struct Odometry
    {
        float x;
        float y;
        float theta;
    };

    Odometry odometry;

    /**
     * @brief 构造函数
     * @note 构造时自动初始化计算值为0
     */
    Odometer(LKMotor& LMotor, LKMotor& RMotor) : LMotor(LMotor), RMotor(RMotor)
    {
        odometry.x = 0;
        odometry.y = 0;
        odometry.theta = 0;
    }

    /**
     * @brief 析构函数
     */
    ~Odometer()
    {}

    //里程计初始化，记录电机上电时的position
    float init()
    {
        clear();
        L_last_position = LMotor.motorFeedback.positionFdb;
        R_last_position = RMotor.motorFeedback.positionFdb;
        DisplacementSet = (L_last_position + R_last_position) / 2;
        return DisplacementSet;
    }

    //清空里程计
    void clear()
    {
        L_last_position = 0;
        R_last_position = 0;
        L_delta_position = 0;
        R_delta_position = 0;
        delta_x = 0;
        delta_theta = 0;

        odometry.x = 0;
        odometry.y = 0;
        odometry.theta = 0;
    }

    //里程计更新
    void update()
    {
        // Get current wheel angular positions
        L_delta_position = LMotor.motorFeedback.positionFdb - L_last_position;
        R_delta_position = RMotor.motorFeedback.positionFdb - R_last_position;

        // Update the last position
        L_last_position = LMotor.motorFeedback.positionFdb;
        R_last_position = RMotor.motorFeedback.positionFdb;

        // Calculate the change in x and theta
        delta_x = (WHEEL_RADIUS * (L_delta_position + R_delta_position)) / 2;
        delta_theta = (WHEEL_RADIUS * (R_delta_position - L_delta_position)) / (BODY_RADIUS*2);

        // Update the X, Y and th values.
        odometry.x += delta_x * cos(odometry.theta);
        odometry.y += delta_x * sin(odometry.theta);
        odometry.theta += delta_theta;
        
        //控制车转角在-pi到pi之间
			if (odometry.theta > Math::Pi) {
            odometry.theta -= 2 * Math::Pi;
        } else if (odometry.theta < -Math::Pi) {
            odometry.theta += 2 * Math::Pi;
        }
    }


};

#endif // ODOMETER_HPP

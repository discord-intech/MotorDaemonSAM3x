//
// Created by discord on 30/09/16.
//

#ifndef MOTORDAEMON_SERVO_HPP
#define MOTORDAEMON_SERVO_HPP

#define SERVO_PWM_TIME_PERIOD 20*1000*1000  // nanosec

#include <dynamixel/DynamixelAx.h>
#include "Motor.hpp"

class Servo
{

public:
    Servo(float, float, float, float, char);
    void initAx();

    void setAngle(double);

private:

    dxlAx ax12;
    char id;
    float lowerBound;
    float lowerAngle;
    float upperBound;
    float upperAngle;
};


#endif //MOTORDAEMON_SERVO_HPP

//
// Created by discord on 30/09/16.
//

#include "dynamixel/Dynamixel.h"
#include "Servo.hpp"


Servo::Servo(float lowB, float lowA, float upB, float upA) : lowerBound(lowB), upperBound(upB), lowerAngle(lowA), upperAngle(upA)
{}

void Servo::initPWM(void)
{


}

void Servo::setAngle(double angle)
{

    //pwm.setDutyPercent(duty);
    if(angle < lowerAngle) angle = lowerAngle;
    if(angle > upperAngle) angle = upperAngle;

    int64_t value = (int64_t) ((upperBound - lowerBound) / (upperAngle - lowerAngle) * (angle + ABS(lowerAngle)) + lowerBound);
   // pwm.setPeriodTime(period, BlackLib::milisecond);


}
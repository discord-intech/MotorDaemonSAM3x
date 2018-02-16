//
// Created by discord on 30/09/16.
//

#include "dynamixel/Dynamixel.h"
#include "Servo.hpp"


Servo::Servo(float lowB, float lowA, float upB, float upA, char id) : lowerBound(lowB), upperBound(upB),
                                                                      lowerAngle(lowA), upperAngle(upA), id(id), ax12(&hdSerial2)
{}

void Servo::initAx(void)
{
    ax12.begin(9600);
    ax12.setStatusReturnLevel(id, 2);
    ax12.setReturnDelayTime(id, 250);
    ax12.setMovingSpeed(id, 512);
    ax12.setMaxTorque(id, 512);
    ax12.setGoalPosition(id, 512);
}

void Servo::setAngle(double angle)
{

    if(angle < lowerAngle) angle = lowerAngle;
    if(angle > upperAngle) angle = upperAngle;

    int16_t value = (int16_t) ((upperBound - lowerBound) / (upperAngle - lowerAngle) * (angle + ABS(lowerAngle)) + lowerBound);
    ax12.setGoalPosition(id, value);


}
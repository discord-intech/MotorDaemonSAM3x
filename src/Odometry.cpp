//
// Created by discord on 26/09/16.
//

#include "Odometry.hpp"


long Odometry::leftTicks;
long Odometry::rightTicks;
int Odometry::valueAL;
int Odometry::valueBL;
int Odometry::valueAR;
int Odometry::valueBR;


Odometry::Odometry()
{

    Odometry::leftTicks = 0;
    Odometry::rightTicks = 0;
    Odometry::valueAL = 0;
    Odometry::valueBL = 0;
    Odometry::valueAR = 0;
    Odometry::valueBR = 0;

    attachInterrupt(digitalPinToInterrupt(CHAN_AL), Odometry::onTickChanALeft, CHANGE);
    attachInterrupt(digitalPinToInterrupt(CHAN_BL), Odometry::onTickChanBLeft, CHANGE);
    attachInterrupt(digitalPinToInterrupt(CHAN_AR), Odometry::onTickChanARight, CHANGE);
    attachInterrupt(digitalPinToInterrupt(CHAN_BR), Odometry::onTickChanBRight, CHANGE);

    Serial.println("odo ok");
    Serial.flush();

}


long Odometry::getLeftValue() {
    return Odometry::leftTicks;
}

long Odometry::getRightValue() {
    return Odometry::rightTicks;
}

void Odometry::onTickChanALeft(void)
{
    valueAL = digitalRead(CHAN_AL);
    if(valueAL == valueBL)
    {
        ++leftTicks;
    }
}

void Odometry::onTickChanBLeft(void)
{
    valueBL = digitalRead(CHAN_BL);
    if(valueAL == valueBL)
    {
        --leftTicks;
    }
}

void Odometry::onTickChanARight(void)
{
    valueAR = digitalRead(CHAN_AR);
    if(valueAR == valueBR)
    {
        --rightTicks;
    }
}

void Odometry::onTickChanBRight(void)
{
    valueBR = digitalRead(CHAN_BR);
    if(valueAR == valueBR)
    {
        ++rightTicks;
    }
}



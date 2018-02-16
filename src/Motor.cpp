//
// Created by discord on 26/09/16.
//

#include <stdlib.h>
#include <iostream>
#include <wiring_digital.h>
#include <wiring_private.h>
#include <wiring_analog.h>
#include "Motor.hpp"


Motor::Motor(char pwm, int dir1, int dir2, bool inv) : PWMpin(pwm), directionPin(dir1), directionPin2(dir2),
                                                                       inversed(inv), actualDirection(Direction::BACKWARD)
{
    pinMode(RIGHT_MOTOR_PWM_PIN, OUTPUT);
    pinMode(LEFT_MOTOR_PWM_PIN, OUTPUT);
    pinMode(RIGHT_MOTOR_DIR_PIN_1, OUTPUT);
    pinMode(LEFT_MOTOR_DIR_PIN_1, OUTPUT);
    pinMode(RIGHT_MOTOR_DIR_PIN_2, OUTPUT);
    pinMode(LEFT_MOTOR_DIR_PIN_2, OUTPUT);
}

LeftMotor::LeftMotor() : Motor(LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_DIR_PIN_1, LEFT_MOTOR_DIR_PIN_2,  false) {}

RightMotor::RightMotor() : Motor(RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_DIR_PIN_1, RIGHT_MOTOR_DIR_PIN_2, true) {}

void Motor::setDirection(Direction way)
{
    if (actualDirection == way)
    {
        return;
    }

    if(!inversed)
    {
        if (way == Direction::FORWARD) {
            digitalWrite(directionPin, LOW);
            digitalWrite(directionPin2, HIGH);
        } else {
            digitalWrite(directionPin2, LOW);
            digitalWrite(directionPin, HIGH);
        }
    }
    else
    {
        if (way == Direction::FORWARD) {
            digitalWrite(directionPin2, LOW);
            digitalWrite(directionPin, HIGH);
        } else {
            digitalWrite(directionPin, LOW);
            digitalWrite(directionPin2, HIGH);
        }
    }

    actualDirection = way;
}

void Motor::initPWM()
{
    setDirection(Direction::FORWARD);

    if(invertedPWM)
    {
        analogWrite(PWMpin, HIGH);
    }
    else
    {
        analogWrite(PWMpin, LOW);
    }

}


void Motor::run(int duty) //duty € [-255;255]
{

    if(duty == this->actualDuty)
    {
        return;
    }

    if(duty < -255)
    {
        duty = -255;
    }

    if(duty > 255)
    {
        duty = 255;
    }

    if(duty >= 0)
    {
        setDirection(Direction::FORWARD);
    }
    else
    {
        setDirection(Direction::BACKWARD);
    }

    analogWrite(PWMpin, (uint8_t)ABS(duty));

    this->actualDuty = duty;
}

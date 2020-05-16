/*******************************************************************************
Monster Motor Shield library

https://www.sparkfun.com/products/10182

Copyright (C) 2017 Juan Menendez <juanmb@gmail.com>
*******************************************************************************/

#include <Arduino.h>
#include "motor.h"


DCMotor::DCMotor(int pin1, int pin2)
{
    DCMotor::pin1 = pin1;
    DCMotor::pin2 = pin2;
    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
    stop();
}

void DCMotor::run(bool dir, int pwm)
{
    // PWM not implemented
    digitalWrite(pin1, dir);
    digitalWrite(pin2, !dir);
}

void DCMotor::stop()
{
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
}

void DCMotor::brake()
{
    stop();
}

bool DCMotor::isRunning()
{
    return digitalRead(pin1) || digitalRead(pin2);
}

int DCMotor::readCurrent()
{
    // Not implemented
    return 0;
}

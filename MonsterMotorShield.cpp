/*******************************************************************************
Monster Motor Shield library

https://www.sparkfun.com/products/10182

Copyright (C) 2017 Juan Menendez <juanmb@gmail.com>
*******************************************************************************/

#include <Arduino.h>
#include "MonsterMotorShield.h"


/*  VNH2SP30 pin definitions
 xxx[0] controls '1' outputs
 xxx[1] controls '2' outputs */
int inApin[2] = {7, 4};  // INA: Clockwise input
int inBpin[2] = {8, 9}; // INB: Counter-clockwise input
int pwmpin[2] = {5, 6}; // PWM input
int cspin[2] = {2, 3}; // CS: Current sense ANALOG input
int enpin[2] = {0, 1}; // EN: Status of switches output (Analog pin)



Motor::Motor(uint8_t n)
{
    // _nmotor can be only 0 or 1
    _nmotor = (n > 0);
    pinMode(inApin[_nmotor], OUTPUT);
    pinMode(inBpin[_nmotor], OUTPUT);
    pinMode(pwmpin[_nmotor], OUTPUT);
    stop();
}

void Motor::run(bool dir, int pwm)
{
    digitalWrite(inApin[_nmotor], dir);
    digitalWrite(inBpin[_nmotor], !dir);
    analogWrite(pwmpin[_nmotor], pwm);
}

void Motor::stop()
{
    digitalWrite(inApin[_nmotor], LOW);
    digitalWrite(inBpin[_nmotor], LOW);
    analogWrite(pwmpin[_nmotor], 0);
}

void Motor::brake()
{
    digitalWrite(inApin[_nmotor], HIGH);
    digitalWrite(inBpin[_nmotor], HIGH);
    analogWrite(pwmpin[_nmotor], 0);
}

bool Motor::isRunning()
{
    return digitalRead(inApin[_nmotor]) || digitalRead(inBpin[_nmotor]);
}

int Motor::readCurrent()
{
    return analogRead(cspin[_nmotor]);
}

/*******************************************************************************
Monster Motor Shield library

https://www.sparkfun.com/products/10182

Copyright (C) 2017 Juan Menendez <juanmb@gmail.com>
*******************************************************************************/

#ifndef _MonsterMotorShield_h_
#define _MonsterMotorShield_h_

#include <stdlib.h>

// Virtual Motor class
class Motor {
public:
    virtual void run(bool dir, int pwm);
    virtual void stop();
    virtual void brake();
    virtual bool isRunning();
    virtual int readCurrent();
};

// Generic DC motor driver
class DCMotor : public Motor {
public:
    DCMotor(int pin1, int pin2);
    void run(bool dir, int pwm);
    void stop();
    void brake();
    bool isRunning();
    int readCurrent();
private:
    int pin1, pin2;
};

// Monster Motor Shield driver
class MMSMotor : public Motor {
public:
    MMSMotor(uint8_t nmotor);
    void run(bool dir, int pwm);
    void stop();
    void brake();
    bool isRunning();
    int readCurrent();
private:
    uint8_t _nmotor;
};

#endif

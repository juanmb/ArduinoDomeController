/*******************************************************************************
Monster Motor Shield library

https://www.sparkfun.com/products/10182

Copyright (C) 2017 Juan Menendez <juanmb@gmail.com>
*******************************************************************************/

#ifndef _MonsterMotorShield_h_
#define _MonsterMotorShield_h_

#include <stdlib.h>


class Motor {
public:
    Motor(uint8_t nmotor);
    void run(bool dir, int pwm);
    void stop();
    void brake();
    bool isRunning();
    int readCurrent();
private:
    uint8_t _nmotor;
};

#endif

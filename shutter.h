/*******************************************************************************
Shutter class definition

https://www.sparkfun.com/products/10182

Copyright (C) 2017 Juan Menendez <juanmb@gmail.com>
*******************************************************************************/


#ifndef _shutter_h_
#define _shutter_h_

#include "MonsterMotorShield.h"

// Lid states
enum State {
    ST_CLOSED,
    ST_OPENING,
    ST_OPEN,
    ST_CLOSING,
    ST_ABORTED,
    ST_ERROR,
};


enum Action {
    DO_NONE,
    DO_OPEN,
    DO_CLOSE,
    DO_ABORT,
};

class Shutter {
public:
    Shutter(Motor *motor, int closedSwitch, int openSwitch,
            int interSwitch, unsigned long timeout);
    Shutter(Motor *motor, int closedSwitch, int openSwitch,
            unsigned long timeout);
    void open();
    void close();
    void abort();
    void update();
    State getState();
private:
    void initState();
    bool interference();
    Motor *motor;
    State state;
    Action nextAction;
    unsigned long runTimeout;
    int swClosed;
    int swOpen;
    int swInter;
};

#endif

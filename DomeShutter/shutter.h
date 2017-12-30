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

// Define a pointer to a function for checking shutter/flap interference
typedef bool (*interFn)(State st);


class Shutter {
public:
    Shutter(Motor *motor, int closedSwitch, int openSwitch, unsigned long timeout,
            interFn checkInterference);
    void open();
    void close();
    void abort();
    void update();
    State getState();
private:
    interFn interference;
    void initState();
    Motor *motor;
    State state;
    Action nextAction;
    unsigned long runTimeout;
    int swClosed;
    int swOpen;
};

#endif

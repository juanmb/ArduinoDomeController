/*******************************************************************************
Monster Motor Shield library

https://www.sparkfun.com/products/10182

Copyright (C) 2017 Juan Menendez <juanmb@gmail.com>
*******************************************************************************/

#include <Arduino.h>
#include "shutter.h"
#include <util/atomic.h>

#define MOTOR_OPEN 0
#define MOTOR_CLOSE 1
#define SPEED 1023
#define DEFAULT_TIMEOUT 30000 // shutter timeout (in ms)

// Shutter constructor.
// motor: pointer to an instance of Motor
// sw1: Limit switch (closed)
// sw2: Limit switch (fully open)
// swInt: Interference switch
Shutter::Shutter(Motor *motorPtr, int closedSwitch, int openSwitch,
        unsigned long timeout, interFn checkInterference)
{
    motor = motorPtr;
    swClosed = closedSwitch;
    swOpen = openSwitch;
    runTimeout = timeout;
    interference = checkInterference;
    nextAction = DO_NONE;
    initState();
}


void Shutter::initState()
{
    if (digitalRead(swClosed))
        state = ST_CLOSED;
    else if (digitalRead(swOpen))
        state = ST_OPEN;
    else
        state = ST_ABORTED;
}


void Shutter::open() { nextAction = DO_OPEN; }
void Shutter::close() { nextAction = DO_CLOSE; }
void Shutter::abort() { nextAction = DO_ABORT; }

State Shutter::getState() { return state; }


// Shutter state machine
void Shutter::update()
{
    Action action;
    static unsigned long t0;

    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        action = nextAction;
        nextAction = DO_NONE;
    }

    switch (state) {
    case ST_CLOSED:
        if (action == DO_OPEN) {
            t0 = millis();
            state = ST_OPENING;
        }
        break;
    case ST_OPEN:
        if (action == DO_CLOSE) {
            t0 = millis();
            state = ST_CLOSING;
        }
        break;
    case ST_ABORTED:
    case ST_ERROR:
        if (action == DO_OPEN) {
            t0 = millis();
            state = ST_OPENING;
        } else if (action == DO_CLOSE) {
            t0 = millis();
            state = ST_CLOSING;
        }
        break;
    case ST_OPENING:
        if (interference(state))
            motor->brake();
        else
            motor->run(MOTOR_OPEN, SPEED);

        if (digitalRead(swOpen)) {
            state = ST_OPEN;
            motor->brake();
        } else if (action == DO_ABORT || action == DO_OPEN) {
            state = ST_ABORTED;
            motor->brake();
        } else if (action == DO_CLOSE) {
            state = ST_CLOSING;
        } else if (millis() - t0 > runTimeout) {
            state = ST_ERROR;
            motor->brake();
        }
        break;
    case ST_CLOSING:
        if (interference(state))
            motor->brake();
        else
            motor->run(MOTOR_CLOSE, SPEED);

        if (digitalRead(swClosed)) {
            state = ST_CLOSED;
            motor->brake();
        } else if (action == DO_ABORT || action == DO_CLOSE) {
            state = ST_ABORTED;
            motor->brake();
        } else if (action == DO_OPEN) {
            state = ST_OPENING;
        } else if (millis() - t0 > runTimeout) {
            state = ST_ERROR;
            motor->brake();
        }
        break;
    }
}

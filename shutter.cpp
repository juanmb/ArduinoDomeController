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
#define SPEED 1024
#define DEFAULT_TIMEOUT 30000 // shutter timeout (in ms)

// Shutter constructor.
// motor: pointer to an instance of Motor
// sw1: Limit switch (closed)
// sw2: Limit switch (fully open)
// swInt: Interference switch
Shutter::Shutter(Motor *motorPtr, int closedSwitch, int openSwitch,
        int interSwitch, unsigned long timeout)
{
    motor = motorPtr;
    swClosed = closedSwitch;
    swOpen = openSwitch;
    swInter = interSwitch;
    nextAction = DO_NONE;
    runTimeout = timeout;
    initState();
}


Shutter::Shutter(Motor *motor, int closedSwitch, int openSwitch,
        unsigned long timeout)
{
    Shutter(motor, closedSwitch, openSwitch, -1, timeout);
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


// Detect mechanical interfence between the two shutters
bool Shutter::interference()
{
    return swInter >= 0 && digitalRead(swInter);
}


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
            motor->run(MOTOR_OPEN, SPEED);
        }
        break;
    case ST_OPEN:
        if (action == DO_CLOSE && !interference()) {
            t0 = millis();
            state = ST_CLOSING;
            motor->run(MOTOR_CLOSE, SPEED);
        }
        break;
    case ST_ABORTED:
    case ST_ERROR:
        if (action == DO_OPEN) {
            t0 = millis();
            state = ST_OPENING;
            motor->run(MOTOR_OPEN, SPEED);
        } else if (action == DO_CLOSE) {
            t0 = millis();
            state = ST_CLOSING;
            motor->run(MOTOR_CLOSE, SPEED);
        }
        break;
    case ST_OPENING:
        if (digitalRead(swOpen)) {
            state = ST_OPEN;
            motor->stop();
        } else if (action == DO_ABORT || action == DO_OPEN || action == DO_CLOSE) {
            state = ST_ABORTED;
            motor->stop();
        } else if (millis() - t0 > runTimeout) {
            state = ST_ERROR;
            motor->stop();
        }
        break;
    case ST_CLOSING:
        if (digitalRead(swClosed)) {
            state = ST_CLOSED;
            motor->stop();
        } else if (action == DO_ABORT || action == DO_OPEN ||
                   action == DO_CLOSE || interference()) {
            state = ST_ABORTED;
            motor->stop();
        } else if (millis() - t0 > runTimeout) {
            state = ST_ERROR;
            motor->stop();
        }
        break;
    }
}

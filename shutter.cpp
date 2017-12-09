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
        int interSwitch, unsigned long timeout)
{
    motor = motorPtr;
    swClosed = closedSwitch;
    swOpen = openSwitch;
    swInter = interSwitch;
    runTimeout = timeout;
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


// Detect mechanical interfence between the two shutters
inline bool Shutter::interference()
{
    return swInter < 0 ? false : digitalRead(swInter);
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
            if (interference())
                state = ST_OPENING_BLOCKED;
            else {
                t0 = millis();
                state = ST_OPENING;
                motor->run(MOTOR_OPEN, SPEED);
            }
        }
        break;
    case ST_OPEN:
        if (action == DO_CLOSE && !interference()) {
            if (interference())
                state = ST_CLOSING_BLOCKED;
            else {
                t0 = millis();
                state = ST_CLOSING;
                motor->run(MOTOR_CLOSE, SPEED);
            }
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
    case ST_OPENING_BLOCKED:
        if (!interference()) {
            t0 = millis();
            state = ST_OPENING;
            motor->run(MOTOR_OPEN, SPEED);
        }
        break;
    case ST_CLOSING_BLOCKED:
        if (!interference()) {
            t0 = millis();
            state = ST_CLOSING;
            motor->run(MOTOR_CLOSE, SPEED);
        }
        break;
    case ST_OPENING:
        if (digitalRead(swOpen)) {
            state = ST_OPEN;
            motor->abort();
        } else if (interference()) {
            state = ST_OPENING_BLOCKED;
            motor->abort();
        } else if (action == DO_ABORT || action == DO_OPEN) {
            state = ST_ABORTED;
            motor->abort();
        } else if (action == DO_CLOSE) {
            state = ST_CLOSING;
            motor->run(MOTOR_CLOSE, SPEED);
        } else if (millis() - t0 > runTimeout) {
            state = ST_ERROR;
            motor->abort();
        }
        break;
    case ST_CLOSING:
        if (digitalRead(swClosed)) {
            state = ST_CLOSED;
            motor->abort();
        } else if (interference()) {
            state = ST_CLOSING_BLOCKED;
            motor->abort();
        } else if (action == DO_ABORT || action == DO_CLOSE) {
            state = ST_ABORTED;
            motor->abort();
        } else if (action == DO_OPEN) {
            state = ST_OPENING;
            motor->run(MOTOR_OPEN, SPEED);
        } else if (millis() - t0 > runTimeout) {
            state = ST_ERROR;
            motor->abort();
        }
        break;
    default:
        motor->abort();
        state = ST_ERROR;
    }
}

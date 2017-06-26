/*******************************************************************************
Monster Motor Shield library

https://www.sparkfun.com/products/10182

Copyright (C) 2017 Juan Menendez <juanmb@gmail.com>
*******************************************************************************/

#include <Arduino.h>
#include "shutter.h"

#define MOTOR_OPEN 0
#define MOTOR_CLOSE 1
#define SPEED 1024

/* Shutter constructor.
 * motor: pointer to an instance of Motor
 * sw1: Limit switch (closed)
 * sw2: Limit switch (fully open)
 * swInt: Interference switch
 */
Shutter::Shutter(Motor *motor, int sw1, int sw2, int swInt)
{
  // _nmotor can be only 0 or 1
  _motor = motor;
  _sw1 = sw1;
  _sw2 = sw2;
  _swInt = swInt;
  _action = DO_NONE;
  initState();
}


Shutter::Shutter(Motor *motor, int sw1, int sw2)
{
  Shutter(motor, sw1, sw2, -1);
}


void Shutter::open()
{
  _action = DO_OPEN;
}


void Shutter::close()
{
  _action = DO_CLOSE;
}


void Shutter::stop()
{
  _action = DO_STOP;
}


state Shutter::getState()
{
  return _state;
}


void Shutter::initState()
{
  if (digitalRead(_sw1))
    _state = ST_CLOSED;
  else if (digitalRead(_sw2))
    _state = ST_OPEN;
  else
    _state = ST_STOPPED;
}


// Detect mechanical interfence between the two shutters
bool Shutter::interference()
{
  return _swInt >= 0 && digitalRead(_swInt);
}


// Shutter state machine
void Shutter::update()
{
  noInterrupts();

  switch (_state) {
  case ST_CLOSED:
    if (_action == DO_OPEN) {
      _state = ST_OPENING;
      _motor->run(MOTOR_OPEN, SPEED);
    }
    break;
  case ST_OPENING:
    if (digitalRead(_sw2)) {
      _state = ST_OPEN;
      _motor->stop();
    } else if (_action == DO_STOP || _action == DO_OPEN || _action == DO_CLOSE) {
      _state = ST_STOPPED;
      _motor->stop();
    }
    break;
  case ST_STOPPED:
    if (_action == DO_OPEN) {
      _state = ST_OPENING;
      _motor->run(MOTOR_OPEN, SPEED);
    } else if (_action == DO_CLOSE) {
      _state = ST_CLOSING;
      _motor->run(MOTOR_CLOSE, SPEED);
    }
    break;
  case ST_CLOSING:
    if (digitalRead(_sw1)) {
      _state = ST_CLOSED;
      _motor->stop();
    } else if (_action == DO_STOP || _action == DO_OPEN ||
               _action == DO_CLOSE || interference()) {
      _state = ST_STOPPED;
      _motor->stop();
    }
    break;
  case ST_OPEN:
    if (_action == DO_CLOSE && !interference()) {
      _state = ST_CLOSING;
      _motor->run(MOTOR_CLOSE, SPEED);
    }
    break;
  }

  _action = DO_NONE;
  interrupts();
}

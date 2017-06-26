/*******************************************************************************
Monster Motor Shield library

https://www.sparkfun.com/products/10182

Copyright (C) 2017 Juan Menendez <juanmb@gmail.com>
*******************************************************************************/

#include <Arduino.h>
#include "shutter.h"


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


state Shutter::readState()
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


void Shutter::update()
{
  noInterrupts();

  switch (_state) {
  case ST_CLOSED:
    if (_action == DO_OPEN) {
      _state = ST_OPENING;
      _motor->run(true, 1024);
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
      _motor->run(true, 1024);
    } else if (_action == DO_CLOSE) {
      _state = ST_CLOSING;
      _motor->run(false, 1024);
    }
    break;
  case ST_CLOSING:
    if (digitalRead(_sw1)) {
      _state = ST_CLOSED;
      _motor->stop();
    } else if (_action == DO_STOP || _action == DO_OPEN || _action == DO_CLOSE) {
      _state = ST_STOPPED;
      _motor->stop();
    }
    break;
  case ST_WAITING:
    break;
  case ST_OPEN:
    if (_action == DO_CLOSE) {
      _state = ST_CLOSING;
      _motor->run(false, 1024);
    }
    break;
  }

  _action = DO_NONE;
  interrupts();
}

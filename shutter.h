/*******************************************************************************
Shutter class definition

https://www.sparkfun.com/products/10182

Copyright (C) 2017 Juan Menendez <juanmb@gmail.com>
*******************************************************************************/


#ifndef _shutter_h_
#define _shutter_h_

#include "MonsterMotorShield.h"

// Lid states
typedef enum state {
  ST_CLOSED,
  ST_OPENING,
  ST_STOPPED,
  ST_CLOSING,
  ST_OPEN,
} state;


typedef enum action {
  DO_NONE,
  DO_OPEN,
  DO_CLOSE,
  DO_STOP,
} action;

class Shutter {
public:
  Shutter(Motor *motor, int sw1, int sw2, int swInt);
  Shutter(Motor *motor, int sw1, int sw2);
  void open();
  void close();
  void stop();
  void update();
  state getState();
private:
  void initState();
  bool interference();
  Motor *_motor;
  state _state;
  action _action;
  int _sw1;
  int _sw2;
  int _swInt;
};

#endif

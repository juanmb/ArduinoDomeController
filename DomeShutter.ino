/*******************************************************************************
ArduinoDomeShutter
Shutter controller for an amateur astronomical dome using Arduino


The MIT License

Copyright (C) 2017 Juan Menendez <juanmb@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
*******************************************************************************/

#include "MonsterMotorShield.h"

// Pin definitions
#define LED      13
#define SW_A1    2
#define SW_A2    3
#define SW_B1    10
#define SW_B2    11
#define SW_INTER 12
#define BUTTONS A4

#define NBUTTONS 4

// Lid states
enum {
  CLOSED,
  OPENING,
  STOPPED,
  CLOSING,
  WAITING,
  OPEN,
} state;

enum {
  BTN_NONE,
  BTN_A_OPEN,
  BTN_A_CLOSE,
  BTN_B_OPEN,
  BTN_B_CLOSE,
};

int buttonLimits[] = {92, 303, 518, 820};


Motor motorA(0);
Motor motorB(1);


int readButton()
{
  int val = analogRead(BUTTONS);

  for (int i = 0; i < NBUTTONS; i++) {
    if (val < buttonLimits[i]) {
      return i + 1;
    }
  }
  return 0;
}


void setup()
{
  Serial.begin(9600);
  pinMode(LED, OUTPUT);
}


void loop()
{
  delay(100);

  switch(readButton()) {
  case BTN_A_OPEN:
    motorA.run(1, 1000);
    digitalWrite(LED, HIGH);
    break;
  case BTN_A_CLOSE:
    motorA.run(0, 1000);
    digitalWrite(LED, HIGH);
    break;
  case BTN_B_OPEN:
    motorB.run(1, 1000);
    digitalWrite(LED, HIGH);
    break;
  case BTN_B_CLOSE:
    motorB.run(0, 1000);
    digitalWrite(LED, HIGH);
    break;
  default:
    motorA.stop();
    motorB.stop();
    digitalWrite(LED, LOW);
    break;
  }
}

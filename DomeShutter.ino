/*******************************************************************************
ArduinoDomeShutter
Shutter controller for an astronomical dome using Arduino


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

#include <avr/wdt.h>
#include "MonsterMotorShield.h"
#include "SerialCommand.h"
#include "shutter.h"

// Pin definitions
#define LED_ERR  13     // error LED
#define SW_A1    12     // shutter closed switch
#define SW_A2    11     // shutter open switch
#define SW_B1    10     // flap closed switch
#define SW_B2    3      // flap open switch
#define SW_INTER 2      // shutter interference detection switch
#define BUTTONS A4      // analog input for reading the buttons

#define NBUTTONS 4

// Timeouts in ms
#define COMMAND_TIMEOUT 5000    // Max. time from last command
#define SHUTTER_TIMEOUT 3000    // Max. time the shutter takes to open/close
#define FLAP_TIMEOUT 3000       // Max. time the flap takes to open/close

enum {
    BTN_NONE,
    BTN_A_OPEN,
    BTN_A_CLOSE,
    BTN_B_OPEN,
    BTN_B_CLOSE,
};

Motor motorA(0);
Motor motorB(1);
Shutter shutter(&motorA, SW_A1, SW_A2, -1, SHUTTER_TIMEOUT);
Shutter flap(&motorB, SW_B1, SW_B2, SW_INTER, FLAP_TIMEOUT);
SerialCommand sCmd;
unsigned long lastCmdTime = 0;

// Detect a pressed button by reading an analog input.
// Every button puts a different voltage at the input.
int readButton()
{
    int buttonLimits[] = {92, 303, 518, 820};
    int val = analogRead(BUTTONS);

    for (int i = 0; i < NBUTTONS; i++) {
        if (val < buttonLimits[i]) {
            return i + 1;
        }
    }
    return 0;
}

void cmdOpenShutter() {
    lastCmdTime = millis();
    shutter.open();
}

void cmdOpenBoth()
{
    lastCmdTime = millis();
    shutter.open();
    flap.open();
}

void cmdClose()
{
    lastCmdTime = millis();
    shutter.close();
    flap.close();
}

void cmdAbort()
{
    lastCmdTime = millis();
    shutter.abort();
    flap.abort();
}

void cmdExit()
{
    lastCmdTime = 0;
    shutter.close();
    flap.close();
}

void cmdStatus()
{
    lastCmdTime = millis();

    State sst = shutter.getState();
    State fst = flap.getState();
    State st = ST_ABORTED;

    if (sst == ST_ERROR || fst == ST_ERROR)
        st = ST_ERROR;
    else if (sst == ST_OPENING || fst == ST_OPENING ||
            sst == ST_OPENING_BLOCKED || fst == ST_OPENING_BLOCKED)
        st = ST_OPENING;
    else if (sst == ST_CLOSING || fst == ST_CLOSING ||
            sst == ST_CLOSING_BLOCKED || fst == ST_CLOSING_BLOCKED)
        st = ST_CLOSING;
    else if (sst == ST_OPEN || fst == ST_OPEN)
        st = ST_OPEN;
    else if (sst == ST_CLOSED && fst == ST_CLOSED)
        st = ST_CLOSED;

    Serial.write('0' + st);
}

void setup()
{
    wdt_disable();
    wdt_enable(WDTO_1S);

    pinMode(LED_ERR, OUTPUT);

    // Map serial commands to functions
    sCmd.addCommand("open", cmdOpenBoth);
    sCmd.addCommand("open1", cmdOpenShutter);
    sCmd.addCommand("close", cmdClose);
    sCmd.addCommand("abort", cmdAbort);
    sCmd.addCommand("exit", cmdExit);
    sCmd.addCommand("stat", cmdStatus);

    Serial.begin(9600);

    digitalWrite(LED_ERR, HIGH);
    delay(200);
    digitalWrite(LED_ERR, LOW);
}


void loop()
{
    int btn = readButton();
    static int btn_prev = 0;
    static int btn_count = 0;

    if (btn && (btn == btn_prev))
        btn_count++;
    else
        btn_count = 0;
    btn_prev = btn;

    if (btn_count == 80) {
        switch(btn) {
        case BTN_A_OPEN:
            shutter.open();
            break;
        case BTN_A_CLOSE:
            shutter.close();
            break;
        case BTN_B_OPEN:
            flap.open();
            break;
        case BTN_B_CLOSE:
            flap.close();
            break;
        }
    }

    shutter.update();
    flap.update();
    sCmd.readSerial();

    int err = (shutter.getState() == ST_ERROR) || (flap.getState() == ST_ERROR);
    digitalWrite(LED_ERR, err);

    // Close the dome if time from last command > COMMAND_TIMEOUT
    if (lastCmdTime && (millis() - lastCmdTime > COMMAND_TIMEOUT)) {
        shutter.close();
        flap.close();
    }

    wdt_reset();
}

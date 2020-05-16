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
#define SW_A1    12     // shutter closed switch (NC)
#define SW_A2    11     // shutter open switch (NO)
#define SW_B1    10     // flap closed switch (NC)
#define SW_B2    3      // flap open switch (NO)
#define SW_INTER 2      // shutter interference detection switch (NC)
#define BUTTONS  A4     // analog input for reading the buttons
#define VBAT_PIN A5     // battery voltage reading

#define BUTTON_REPS 80  // Number of ADC readings required to detect a pressed button

// Timeouts in ms
#define COMMAND_TIMEOUT 60000   // Max. time from last command
#define SHUT_TIMEOUT 75000   // Max. time the shutter takes to open/close
#define FLAP_TIMEOUT 15000   // Max. time the flap takes to open/close

#define LEN(a) ((int)(sizeof(a)/sizeof(*a)))

enum {
    BTN_NONE,
    BTN_A_OPEN,
    BTN_A_CLOSE,
    BTN_B_OPEN,
    BTN_B_CLOSE,
};

// Detect mechanical interfence between the two shutters
bool checkFlapInterference(State st)
{
    return (st == ST_OPENING) && digitalRead(SW_INTER);
}

// Detect mechanical interfence between the two shutters
bool checkShutInterference(State st)
{
    return (st == ST_CLOSING) && digitalRead(SW_INTER) && !digitalRead(SW_B1);
}


// Monster Motor Shield
MMSMotor motorA(0);
MMSMotor motorB(1);

Shutter shutters[] = {
    Shutter(&motorA, SW_A1, SW_A2, SHUT_TIMEOUT, checkShutInterference),
    Shutter(&motorB, SW_B1, SW_B2, FLAP_TIMEOUT, checkFlapInterference),
};

SerialCommand sCmd;
unsigned long lastCmdTime = 0;

// Detect a pressed button by reading an analog input.
// Every button puts a different voltage at the input.
int readButton()
{
    int buttonLimits[] = {92, 303, 518, 820};
    int val = analogRead(BUTTONS);

    for (int i = 0; i < 4; i++) {
        if (val < buttonLimits[i]) {
            return i + 1;
        }
    }
    return 0;
}

// Return the combined status of the shutters
State domeStatus()
{
    bool closed = true;
    State st;

    for (int i = 0; i < LEN(shutters); i++) {
        st = shutters[i].getState();
        closed = closed && (st == ST_CLOSED);

        if (st == ST_ERROR)
            return ST_ERROR;
        else if (st == ST_OPENING)
            return ST_OPENING;
        else if (st == ST_CLOSING)
            return ST_CLOSING;
        else if (st == ST_OPEN)
            return ST_OPEN;
    }
    if (closed)
        return ST_CLOSED;
    return ST_ABORTED;
}

// Open main shutter only
void cmdOpenShutter()
{
    lastCmdTime = millis();
    shutters[0].open();
}

// Open shutters
void cmdOpenBoth()
{
    lastCmdTime = millis();
    for (int i = 0; i < LEN(shutters); i++)
        shutters[i].open();
}

// Close shutters
void cmdClose()
{
    lastCmdTime = millis();
    for (int i = 0; i < LEN(shutters); i++)
        shutters[i].close();
}

void cmdAbort()
{
    lastCmdTime = millis();
    for (int i = 0; i < LEN(shutters); i++)
        shutters[i].abort();
}

void cmdExit()
{
    lastCmdTime = 0;
    for (int i = 0; i < LEN(shutters); i++)
        shutters[i].close();
}

void cmdStatus()
{
    lastCmdTime = millis();
    State st = domeStatus();
    Serial.write('0' + st);
}

void cmdGetVBat()
{
    lastCmdTime = millis();
    int val = analogRead(VBAT_PIN);
    char buffer[8];
    sprintf(buffer, "v%04d", val);
    Serial.write(buffer);
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
    sCmd.addCommand("vbat", cmdGetVBat);

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

    if (btn_count == BUTTON_REPS) {
        switch(btn) {
        case BTN_A_OPEN:
            shutters[0].open();
            break;
        case BTN_A_CLOSE:
            shutters[0].close();
            break;
        case BTN_B_OPEN:
            if (LEN(shutters) > 1)
                shutters[1].open();
            break;
        case BTN_B_CLOSE:
            if (LEN(shutters) > 1)
                shutters[1].close();
            break;
        }
    }

    State st = domeStatus();

    // Close the dome if time from last command > COMMAND_TIMEOUT
    if ((lastCmdTime > 0) && ((millis() - lastCmdTime) > COMMAND_TIMEOUT)) {
        if (st != ST_CLOSED) {
            lastCmdTime = 0;
            for (int i = 0; i < LEN(shutters); i++)
                shutters[i].close();
        }
    }

    digitalWrite(LED_ERR, (st == ST_ERROR));

    for (int i = 0; i < LEN(shutters); i++)
        shutters[i].update();

    sCmd.readSerial();
    wdt_reset();
}

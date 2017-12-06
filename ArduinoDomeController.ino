/******************************************************************
    Author:     Juan Menendez Blanco    <juanmb@gmail.com>

    This code is part of the ArduinoDomeController project:
        https://github.com/juanmb/ArduinoDomeController

*******************************************************************/

#include <SoftwareSerial.h>
#include "serial_command.h"


#define BAUDRATE 19200

// pin definitions
#define HC12_RX 4       // Recieve Pin on HC12
#define HC12_TX 5       // Transmit Pin on HC12
#define ENCODER1 2      // Azimuth encoder
#define ENCODER2 3      // Azimuth encoder
#define MOTOR_JOG 7     // Motor jog mode (low speed)
#define MOTOR_CW 8      // Move motor clockwise
#define MOTOR_CCW 9     // Move motor counterclockwise
#define HALL_PROBE 10   // Home probe

// Message Destination
#define TO_MAXDOME  0x00
#define TO_COMPUTER 0x80

// Commands
#define ABORT_CMD   0x03 // Abort azimuth movement
#define HOME_CMD    0x04 // Move until 'home' position is detected
#define GOTO_CMD    0x05 // Go to azimuth position
#define SHUTTER_CMD 0x06 // Send a command to shutter
#define STATUS_CMD  0x07 // Retrieve status
#define TICKS_CMD   0x09 // Set the number of tick per revolution of the dome
#define ACK_CMD     0x0A // ACK (?)
#define SETPARK_CMD 0x0B // Set park coordinates and shutter closing policy

// Shutter commands
#define OPEN_SHUTTER            0x01
#define OPEN_UPPER_ONLY_SHUTTER 0x02
#define CLOSE_SHUTTER           0x03
#define EXIT_SHUTTER            0x04 // Command sent to shutter on program exit
#define ABORT_SHUTTER           0x07

#define DIR_CW  0x01
#define DIR_CCW 0x01

#define AZ_TOLERANCE    4       // Azimuth target tolerance in encoder ticks
#define AZ_SLOW_RANGE   16      //
#define AZ_TIMEOUT      30000   // Azimuth movement timeout


enum MotorSpeed { MOTOR_STOP, MOTOR_SLOW, MOTOR_FAST };

enum AzimuthEvent {
    EVT_NONE,
    EVT_GOTO,
    EVT_HOME,
    EVT_ABORT
};

enum AzimuthStatus {
    ST_IDLE,
    ST_GOING,
    ST_GOING_SLOW,
    ST_HOMING,
    ST_ERROR,
};

// MaxDome II azimuth status
enum MDAzimuthStatus {
    AS_IDLE = 1,
    AS_MOVING_CW,
    AS_MOVING_CCW,
    AS_IDLE2,
    AS_ERROR
};

// MaxDome II shutter status
enum ShutterStatus {
    SS_CLOSED = 0,
    SS_OPENING,
    SS_OPEN,
    SS_CLOSING,
    SS_ABORTED,
    SS_ERROR
};


SoftwareSerial HC12(HC12_TX, HC12_RX); // Create Software Serial Port
SerialCommand sCmd;

bool home_reached = false;
bool park_on_shutter = false;   // Park before closing the shutters
uint8_t current_dir = DIR_CW;   // Current azimuth movement direction
uint16_t current_pos = 0;       // Current dome position
uint16_t target_pos = 0;        // Target dome position
uint16_t home_pos = 0;          // Home position
uint16_t park_pos = 0;          // Parking position
uint16_t ticks_per_turn = 360;  // Encoder ticks per dome revolution
AzimuthStatus state = ST_IDLE;
AzimuthEvent az_event = EVT_NONE;


uint16_t bytesToInt(uint8_t *data) {
    uint16_t out = data[0] & 0xff;
    out <<= 8;
    out |= data[1] & 0xff;
    return out;
}

void intToBytes(uint16_t value, uint8_t *data) {
    data[0] = (value >> 8) & 0xff;
    data[1] = value & 0xff;
}


// Obtain the direction of the shortest path between two positons
uint8_t getDirection(uint16_t current, uint16_t target)
{
    if (target > current)
        return (target - current > ticks_per_turn/2) ? DIR_CCW : DIR_CW;

    return (current - target > ticks_per_turn/2) ? DIR_CW : DIR_CCW;
}

// Obtain the distance between two positons
uint16_t getDistance(uint16_t current, uint16_t target)
{
    // obtain the absolute value of the difference
    uint16_t diff = (target > current) ?  target - current : current - target;

    if (diff > ticks_per_turn/2)
        return ticks_per_turn - diff;

    return diff;
}

inline void moveAzimuth(uint8_t dir)
{
    digitalWrite(MOTOR_CW, dir == DIR_CW);
    digitalWrite(MOTOR_CCW, dir != DIR_CW);
}

inline void stopAzimuth()
{
    digitalWrite(MOTOR_JOG, LOW);
    digitalWrite(MOTOR_CW, LOW);
    digitalWrite(MOTOR_CCW, LOW);
}

inline void setJog(bool enable)
{
    digitalWrite(MOTOR_JOG, enable);
}

ShutterStatus getShutterStatus()
{
    HC12.println("status");

    // TODO
    //while (HC12.available() > 0) {
        //delay(10);
        //c = HC12.read();
    //}
    return SS_CLOSED;
}


void cmdAbortAzimuth(uint8_t *cmd)
{
    az_event = EVT_ABORT;

    uint8_t resp[] = {START, 2, TO_COMPUTER | ABORT_CMD, 0x00};
    sCmd.sendResponse(resp, 4);
}

void cmdHomeAzimuth(uint8_t *cmd)
{
    az_event = EVT_HOME;

    uint8_t resp[] = {START, 2, TO_COMPUTER | HOME_CMD, 0x00};
    current_dir = getDirection(current_pos, home_pos);
    sCmd.sendResponse(resp, 4);
}

void cmdGotoAzimuth(uint8_t *cmd)
{
    current_dir = cmd[3];
    target_pos = bytesToInt(cmd + 4);
    az_event = EVT_GOTO;

    uint8_t resp[] = {START, 2, TO_COMPUTER | GOTO_CMD, 0x00};
    sCmd.sendResponse(resp, 4);
}

void cmdShutterCommand(uint8_t *cmd)
{
    switch(cmd[3]) {
    case OPEN_SHUTTER:
        HC12.println("open1");
        break;
    case OPEN_UPPER_ONLY_SHUTTER:
        HC12.println("open2");
        break;
    case CLOSE_SHUTTER:
        HC12.println("close");
        break;
    case EXIT_SHUTTER:
        HC12.println("exit");
        break;
    case ABORT_SHUTTER:
        HC12.println("abort");
        break;
    }

    uint8_t resp[] = {START, 2, TO_COMPUTER | SHUTTER_CMD, 0x00};
    sCmd.sendResponse(resp, 4);
}

void cmdStatus(uint8_t *cmd)
{
    MDAzimuthStatus az_status;
    if (state == ST_IDLE) {
        az_status = AS_IDLE;
    } else if (state == ST_ERROR) {
        az_status = AS_ERROR;
    } else {
        if (current_dir == DIR_CW)
            az_status = AS_MOVING_CW;
        else
            az_status = AS_MOVING_CCW;
    }

    uint8_t sh_status = (uint8_t)getShutterStatus();
    uint8_t resp[] = {START, 8, TO_COMPUTER | STATUS_CMD, sh_status, az_status,
        0x00, 0x00, 0x00, 0x00, 0x00};

    intToBytes(current_pos, resp + 5);
    intToBytes(home_pos, resp + 7);

    sCmd.sendResponse(resp, 10);
}

void cmdSetPark(uint8_t *cmd)
{
    park_on_shutter = cmd[3];
    park_pos = bytesToInt(cmd + 4);

    uint8_t resp[] = {START, 2, TO_COMPUTER | SETPARK_CMD, 0x00};
    sCmd.sendResponse(resp, 4);
}

void cmdSetTicks(uint8_t *cmd)
{
    ticks_per_turn = bytesToInt(cmd + 3);

    uint8_t resp[] = {START, 2, TO_COMPUTER | TICKS_CMD, 0x00};
    sCmd.sendResponse(resp, 4);
}

void cmdAck(uint8_t *cmd)
{
    uint8_t resp[] = {START, 2, TO_COMPUTER | ACK_CMD, 0x00};
    sCmd.sendResponse(resp, 4);
}

void updateAzimuthFSM()
{
    static unsigned long t0;

    switch(state) {
    case ST_HOMING:
        if (az_event == EVT_ABORT) {
            stopAzimuth();
            state = ST_IDLE;
        } else if (home_reached) {
            stopAzimuth();
            state = ST_IDLE;
            home_reached = false;
        } else if (millis() - t0 > AZ_TIMEOUT) {
            stopAzimuth();
            state = ST_ERROR;
        }
        break;

    case ST_GOING:
        if (az_event == EVT_ABORT) {
            stopAzimuth();
            state = ST_IDLE;
        } else if (getDistance(current_pos, target_pos) < AZ_SLOW_RANGE) {
            moveAzimuth(current_dir);
            setJog(true);
            state = ST_GOING_SLOW;
        } else if (millis() - t0 > AZ_TIMEOUT) {
            stopAzimuth();
            state = ST_ERROR;
        }
        break;

    case ST_GOING_SLOW:
        if (az_event == EVT_ABORT) {
            stopAzimuth();
            state = ST_IDLE;
        } else if (getDistance(current_pos, target_pos) < AZ_TOLERANCE) {
            stopAzimuth();
            state = ST_IDLE;
        } else if (millis() - t0 > AZ_TIMEOUT) {
            stopAzimuth();
            state = ST_ERROR;
        }
        break;

    case ST_ERROR:
    case ST_IDLE:
        if (az_event == EVT_HOME) {
            t0 = millis();
            state = ST_HOMING;
            moveAzimuth(current_dir);
        } else if (az_event == EVT_GOTO) {
            t0 = millis();
            state = ST_GOING;
            moveAzimuth(current_dir);
        }
        break;
    }
    az_event = EVT_NONE;
}

// Encoder interrupt service routine
void encoderISR()
{
    if(digitalRead(ENCODER1) == digitalRead(ENCODER2)) {
        if (current_pos == 0)
            current_pos = ticks_per_turn - 1;
        else
            current_pos--;
    } else {
        if (current_pos >= ticks_per_turn)
            current_pos = 0;
        else
            current_pos++;
    }

    // store detected home position
    if (!digitalRead(HALL_PROBE)) {
        if (state == ST_HOMING) {
            home_pos = 0;
            current_pos = 0;
            home_reached = true;
        } else
            home_pos = current_pos;
    }
}


void setup()
{
    sCmd.addCommand(ABORT_CMD, 2, cmdAbortAzimuth);
    sCmd.addCommand(HOME_CMD, 2, cmdHomeAzimuth);
    sCmd.addCommand(GOTO_CMD, 5, cmdGotoAzimuth);
    sCmd.addCommand(SHUTTER_CMD, 3, cmdShutterCommand);
    sCmd.addCommand(STATUS_CMD, 2, cmdStatus);
    sCmd.addCommand(SETPARK_CMD, 5, cmdSetPark);
    sCmd.addCommand(TICKS_CMD, 4, cmdSetTicks);
    sCmd.addCommand(ACK_CMD, 2, cmdAck);

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(MOTOR_JOG, OUTPUT);
    pinMode(MOTOR_CW, OUTPUT);
    pinMode(MOTOR_CCW, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(ENCODER1), encoderISR, CHANGE);

    Serial.begin(19200);
    HC12.begin(9600);   // Open serial port to HC12
}


void loop()
{
    sCmd.readSerial();
    updateAzimuthFSM();
}

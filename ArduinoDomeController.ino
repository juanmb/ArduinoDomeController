/******************************************************************
    Author:     Juan Menendez Blanco    <juanmb@gmail.com>

    This code is part of the ArduinoDomeController project:
        https://github.com/juanmb/ArduinoDomeController

*******************************************************************/

#include <SoftwareSerial.h>
#include "serial_command.h"


#define BAUDRATE 19200

// pin definitions
#define MOTOR_DIR 2     // Azimuth motor direction
#define MOTOR_ENABLE 3  // Azimuth motor enable
#define HC12_RX 4       // Recieve Pin on HC12
#define HC12_TX 5       // Transmit Pin on HC12
#define ENCODER1 8      // Azimuth encoder
#define ENCODER2 9      // Azimuth encoder
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


SoftwareSerial HC12(HC12_TX, HC12_RX); // Create Software Serial Port
SerialCommand sCmd;


void cmdAbortAzimuth(char *cmd)
{
    char resp[] = {START, 2, TO_COMPUTER | ABORT_CMD, 0x00};
    sCmd.sendResponse(resp, 4);
}

void cmdHomeAzimuth(char *cmd)
{
    char resp[] = {START, 2, TO_COMPUTER | HOME_CMD, 0x00};
    sCmd.sendResponse(resp, 4);
}

void cmdAck(char *cmd)
{
    char resp[] = {START, 2, TO_COMPUTER | ACK_CMD, 0x00};
    sCmd.sendResponse(resp, 4);
}

void setup()
{
    sCmd.addCommand(ABORT_CMD, 2, cmdAbortAzimuth);
    sCmd.addCommand(HOME_CMD, 2, cmdHomeAzimuth);
    //sCmd.addCommand(GOTO_CMD, 5, cmdGotoAzimuth);
    //sCmd.addCommand(SHUTTER_CMD, 3, cmdShutterCommand);
    //sCmd.addCommand(STATUS_CMD, 2, cmdStatus);
    //sCmd.addCommand(SETPARK_CMD, 5, cmdSetPark);
    //sCmd.addCommand(TICKS_CMD, 4, cmdSetTicks);
    sCmd.addCommand(ACK_CMD, 2, cmdAck);

    //pinMode(LED_BUILTIN, OUTPUT);
    //pinMode(MOTOR_DIR, OUTPUT);
    //pinMode(MOTOR_ENABLE, OUTPUT);

    Serial.begin(19200);
    HC12.begin(9600);   // Open serial port to HC12
}


void loop()
{
    sCmd.readSerial();
}

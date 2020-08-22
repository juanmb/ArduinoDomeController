/******************************************************************
    Author:     Juan Menendez Blanco    <juanmb@gmail.com>

    This code is part of the ArduinoDomeController project:
        https://github.com/juanmb/ArduinoDomeController

*******************************************************************/

#include <Arduino.h>
#include "serial_command.h"


SerialCommand::SerialCommand()
{
    nCommands = 0;
    cmdSize = 0;
    bufPos = 0;
}

uint8_t SerialCommand::getCRC(uint8_t *cmd, uint8_t length)
{
    char crc = 0;

    for (int i = 1; i <= length; i++) {
        crc -= cmd[i];
    }
    return crc;
}

int SerialCommand::addCommand(const uint8_t id, uint8_t size, cbFunction function)
{
    if (nCommands >= MAX_COMMANDS) {
        return 1;
    }

    commandList[nCommands].id = id;
    commandList[nCommands].size = size;
    commandList[nCommands].function = function;
    nCommands++;
    return 0;
}

void SerialCommand::readSerial()
{
    while (Serial.available()) {
        char c = Serial.read();

        switch (bufPos) {
        case 0:
            // start byte
            if (c != START)
                continue;
            break;

        case 1:
            // command length
            if (c < 0x02 || c > 0x0e)
                continue;

            cmdSize = c;
            break;

        case 2:
            // command id
            for (int i = 0; i < nCommands; i++) {
                if (commandList[i].id == c) {
                    // check command size
                    if (cmdSize != commandList[i].size) {
                    }
                    cmdFunction = commandList[i].function;
                    break;
                }
            }
            break;
        }

        buffer[bufPos++] = c;

        if (bufPos == cmdSize + 2) {
            //TODO: check CRC
            cmdFunction(buffer);
            bufPos = 0;
        }
    }
}

void SerialCommand::sendResponse(uint8_t *cmd, uint8_t length)
{
    cmd[length - 1] = getCRC(cmd, length - 2);

    for (int i = 0; i < length; i++) {
        Serial.write(cmd[i]);
    }
}

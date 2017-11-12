/******************************************************************
    Author:     Juan Menendez Blanco    <juanmb@gmail.com>

    This code is part of the ArduinoDomeController project:
        https://github.com/juanmb/ArduinoDomeController

*******************************************************************/

#ifndef _SerialCommand_h_
#define _SerialCommand_h_

#include <inttypes.h>

#define MAX_CMD_SIZE 32
#define MAX_COMMANDS 16

#define START 0x01  // Start byte


typedef void (* cbFunction)(char*);


typedef struct commandCallback {
    char id;
    uint8_t size;
    cbFunction function;
} commandCallback;


class SerialCommand {
public:
    SerialCommand();
    void readSerial();
    void sendResponse(char *, uint8_t);
    int addCommand(const char, uint8_t, cbFunction);
private:
    char getCRC(char *, uint8_t);
    commandCallback commandList[MAX_COMMANDS];
    char buffer[MAX_CMD_SIZE];
    cbFunction cmdFunction;
    int nCommands;
    int cmdSize;
    int bufPos;
};

#endif

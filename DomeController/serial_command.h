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


typedef void (* cbFunction)(uint8_t*);


typedef struct commandCallback {
    uint8_t id;
    uint8_t size;
    cbFunction function;
} commandCallback;


class SerialCommand {
public:
    SerialCommand();
    void readSerial();
    void sendResponse(uint8_t *, uint8_t);
    int addCommand(const uint8_t, uint8_t, cbFunction);
private:
    uint8_t getCRC(uint8_t*, uint8_t);
    commandCallback commandList[MAX_COMMANDS];
    uint8_t buffer[MAX_CMD_SIZE];
    cbFunction cmdFunction;
    int nCommands;
    int cmdSize;
    int bufPos;
};

#endif

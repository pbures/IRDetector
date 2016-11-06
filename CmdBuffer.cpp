/*
 * CmdBuffer.cpp
 *
 *  Created on: 6. 11. 2016
 *      Author: pbures
 */

#include "CmdBuffer.h"

CmdBuffer::CmdBuffer() {
	commandsPtr = 0;
	commandsReadPtr = 0;
}

bool CmdBuffer::hasCode() {
	return (commandsPtr == commandsReadPtr);
}

Code *CmdBuffer::getCode() {
	Code* ret = 0;
	if (hasCode()) {
		ret = &(codes[commandsReadPtr]);
		commandsReadPtr = (commandsReadPtr + 1) % COMMANDS_BUFLEN;
	}

	return ret;
}

bool CmdBuffer::add(uint8_t code, uint8_t ch) {
	uint8_t writePtr = (commandsPtr + 1) % COMMANDS_BUFLEN;
	if (writePtr == commandsReadPtr) {
		return false;
	} else {
		commandsPtr = writePtr;
		commands[commandsPtr] = code;

		codes[commandsPtr].code = code;
		codes[commandsPtr].channel = ch;
		return true;
	}
}

void CmdBuffer::reset() {
	commandsReadPtr = commandsPtr;
}

void CmdBuffer::print(char *prefix) {

	if (prefix) {
		printString(prefix);
	}

	uint8_t localReadPtr = commandsReadPtr;
	while (localReadPtr != commandsPtr) {
		printByte(commands[localReadPtr]);
		printString(" ,");
		localReadPtr = (localReadPtr + 1) % COMMANDS_BUFLEN;
	}

	printString("\r\n");
}


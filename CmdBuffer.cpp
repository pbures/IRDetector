/*
 * CmdBuffer.cpp
 *
 *  Created on: 6. 11. 2016
 *      Author: pbures
 */

#include "CmdBuffer.h"

CmdBuffer::CmdBuffer() {
	codesPtr = 0;
	codesReadPtr = 0;
}

bool CmdBuffer::hasCode() {
	if (codesPtr == codesReadPtr) {
		return false;
	} else {
		return true;
	}
}

Code *CmdBuffer::getCode() {
	Code* ret = 0;
	if (hasCode()) {
		ret = &(codes[codesReadPtr]);
		codesReadPtr = (codesReadPtr + 1) % COMMANDS_BUFLEN;
	}

	return ret;
}

bool CmdBuffer::add(uint8_t code, uint8_t ch) {
	codes[codesPtr].code = code;
	codes[codesPtr].channel = ch;
	codesPtr = (codesPtr + 1) % COMMANDS_BUFLEN;

	return true;
}

void CmdBuffer::reset() {
	codesReadPtr = codesPtr;
}

void CmdBuffer::print() {
	uint8_t localReadPtr = codesReadPtr;
	while (localReadPtr != codesPtr) {
		Code* code = getCode();
		printByte(code->channel);
		printString(":");
		printByte(code->code);
		printString(", ");

		localReadPtr = (localReadPtr + 1) % COMMANDS_BUFLEN;
	}
	printString("\r\n");
}


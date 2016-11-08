/*
 * CmdBuffer.cpp
 *
 *  Created on: 6. 11. 2016
 *      Author: pbures
 */

#include "CmdBuffer.h"

CmdBuffer::CmdBuffer() {
	codesPtr = 0;
	newCodes = false;

	for(uint8_t c=0;c<COMMANDS_BUFLEN;c++){
		codes[c].channel = UNDEF_CHANNEL;
	}
}

bool CmdBuffer::add(uint8_t code, uint8_t ch) {
	codes[codesPtr].code = code;
	codes[codesPtr].channel = ch;
	codesPtr = (codesPtr + 1) % COMMANDS_BUFLEN;
	newCodes = true;

	return true;
}

void CmdBuffer::reset() {
	newCodes = false;
}

bool CmdBuffer::hasNewCode() {
	return newCodes;
}

CmdBufferIterator::CmdBufferIterator(CmdBuffer* buffer, bool full) : buffer(buffer) {
	this->buffer = buffer;
	codesReadPtr = buffer->codesPtr;
	if (full) {
		codesReadPtr = (codesReadPtr + 1) % COMMANDS_BUFLEN;
	}
}

Code* CmdBufferIterator::next() {
	Code* ret = 0;
	if (codesReadPtr != buffer->codesPtr) {
		ret = &(buffer->codes[codesReadPtr]);
		codesReadPtr = (codesReadPtr + 1) % COMMANDS_BUFLEN;
	}

	return ret;
}

void CmdBufferIterator::print() {
	Code *code;
	while(code = next()){
		printByte(code->channel);
		printString(":");
		printByte(code->code);
		printString(", ");
	}
}

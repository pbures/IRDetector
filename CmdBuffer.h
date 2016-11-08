/*
 * CmdBuffer.h
 *
 *  Created on: 6. 11. 2016
 *      Author: pbures
 */

#ifndef CMDBUFFER_H_
#define CMDBUFFER_H_

#include "config.h"
#include <stdint.h>
#include "USART/USART.h"

#define COMMANDS_BUFLEN 64

struct Code {
	uint8_t code;
	uint8_t channel;
};

class CmdBufferIterator;

class CmdBuffer {
public:
	CmdBuffer();
	static const uint8_t UNDEF_CHANNEL = 255;
	bool add(uint8_t code, uint8_t ch);

	/* Return true if a new code has been added after reset() has been called. */
	bool hasNewCode();

	/*
	 * Acknowledges the new codes were processed. The hasNewCode() returns false after
	 * this call until a call to add() is made.
	 */
	void reset();

private:
	Code codes[COMMANDS_BUFLEN];
	uint8_t codesPtr;

	bool newCodes;
	friend class CmdBufferIterator;
};

class CmdBufferIterator {
public:
	/*
	 * Creates an iterator over the buffer. If full is false, the iterator iterates only
	 * over a new  values, added after the creation. If full is true, iterator iterates
	 * over whole buffer length.
	 */
	CmdBufferIterator(CmdBuffer* buffer, bool full=false);

	/*
	 * Returns next value in the buffer if available, 0 otherwise.
	 */
	Code* next();

	/*
	 * Simple print of the buffer new values to serial UART.
	 */
	void print();
private:
	uint8_t codesReadPtr;
	CmdBuffer* buffer;
};

#endif /* CMDBUFFER_H_ */

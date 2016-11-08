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

#define COMMANDS_BUFLEN 32
struct Code {
	uint8_t code;
	uint8_t channel;
};

class CmdBuffer {
public:
	CmdBuffer();

	bool hasCode();
	Code* getCode();
	bool add(uint8_t code, uint8_t ch);
	void reset();
	void print();

private:
	uint8_t codesPtr;
	uint8_t codesReadPtr;
	Code codes[COMMANDS_BUFLEN];

	Code retCode;
};

#endif /* CMDBUFFER_H_ */

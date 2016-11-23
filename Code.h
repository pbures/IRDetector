/*
 * Code.h
 *
 *  Created on: 9. 11. 2016
 *      Author: pbures
 */

#ifndef CODE_H_
#define CODE_H_

class Code {
public:
	static const uint8_t UNDEF_CHANNEL = 255;

	void copy(Code *c) {
		code = c->code;
		channel = c->channel;
	}

	Code() {
		channel = UNDEF_CHANNEL;
		code = 0;
	}

	void print() {
		printf("(%d:%d),", channel, code);
	}

	uint8_t code;
	uint8_t channel;
};

#endif /* CODE_H_ */

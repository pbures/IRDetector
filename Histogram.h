/*
 * Histogram.h
 *
 *  Created on: 6. 11. 2016
 *      Author: pbures
 */

#ifndef HISTOGRAM_H_
#define HISTOGRAM_H_

#include "config.h"
#include <stdint.h>

#include "Code.h"
#include "RingBuffer.h"

template<class Code, uint8_t MAXSIZE>
class Histogram {

public:
	Histogram(RingBuffer<Code, MAXSIZE> *buffer) :
			buffer(buffer), numMainChannels(0) {
		reset();
	}

	bool updateHistogram() {
		if (!buffer->hasNewCode())
			return false;

		for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
			channels[ch] = 0;
		}

		Code* code;
		RingBufferIterator<Code, MAXSIZE> iterator(buffer, true);
		while ((code = iterator.next())) {
			if (code->channel == Code::UNDEF_CHANNEL)
				continue;

			channels[code->channel] = min(channels[code->channel] + 1, 255);
		}
		buffer->reset();

		uint8_t mainChannel = 0;
		for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
			mainChannel = max(mainChannel, channels[ch]);
		}

		numMainChannels = 0;
		for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
			if (channels[ch] == mainChannel) {
				mainChannels[numMainChannels++] = ch;
			}
		}

		return true;
	}

	uint8_t* getMainChannels() {
		return mainChannels;
	}

	void print() {
		printString("Histogram:\r\n");

		for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
			printString("CH:");
			printByte(ch);
			printString(":");
			for (uint8_t i = 0; i < channels[ch]; i++) {
				printString("#");
			}
			printString("\r\n");
		}

		if (numMainChannels > 0) {
			printString("Main Channels: [");
			for (uint8_t ch = 0; ch < numMainChannels; ch++) {
				printByte(mainChannels[ch]);
				printString(",");
			}
			printString("]\r\n");
		}

	}

private:
	void reset() {
		for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
			channels[ch] = 0;
		}
	}

	inline uint8_t min(uint8_t a, uint8_t b) {
		return (a < b) ? a : b;
	}

	inline uint8_t max(uint8_t a, uint8_t b) {
		return (a > b) ? a : b;
	}

	RingBuffer<Code, MAXSIZE>* buffer;
	uint8_t numMainChannels;
	uint8_t mainChannels[NUM_CHANNELS];
	uint8_t channels[NUM_CHANNELS];
};

#endif /* HISTOGRAM_H_ */

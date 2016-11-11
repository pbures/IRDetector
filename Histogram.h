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
			buffer(buffer) {
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

		mCh = 0;
		for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
			if (channels[ch] == mainChannel)
				mCh |= (1 << ch);
		}

		return true;
	}

	uint8_t getMainChannels() {
		return mCh;
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

		if (mCh > 0) {
			printString("Main Channels: [");
			for (uint8_t ch = 0; ch < 8; ch++) {
				if (!(mCh & (1 << ch)))
					continue;
				printByte(ch);
				printString(",");
			}
			printString("]\r\n");
		}

	}

private:
	void reset() {
		for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
			channels[ch] = 0;
			mCh = 0;
		}
	}

	inline uint8_t min(uint8_t a, uint8_t b) {
		return (a < b) ? a : b;
	}

	inline uint8_t max(uint8_t a, uint8_t b) {
		return (a > b) ? a : b;
	}

	RingBuffer<Code, MAXSIZE>* buffer;
	uint8_t mCh;
	uint8_t channels[NUM_CHANNELS];
};

#endif /* HISTOGRAM_H_ */

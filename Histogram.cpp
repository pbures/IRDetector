/*
 * Histogram.cpp
 *
 *  Created on: 6. 11. 2016
 *      Author: pbures
 */

#include "Histogram.h"

Histogram::Histogram(CmdBuffer* cmdBuffer) : buffer(cmdBuffer)
{
	reset();
}

void Histogram::reset() {
	for(uint8_t ch=0; ch < NUM_CHANNELS; ch++) {
		channels[ch] = 0;
	}
}

inline uint8_t Histogram::min(uint8_t a, uint8_t b) {
	return (a < b) ? a : b;
}

inline uint8_t Histogram::max(uint8_t a, uint8_t b) {
	return (a > b) ? a : b;
}

uint8_t Histogram::getMaxChannelVal() {
	uint8_t maxVal=0;
	for(uint8_t ch=0; ch < NUM_CHANNELS; ch++){
		if (maxVal < channels[ch]) maxVal = channels[ch];
	}
	return maxVal;
}

uint8_t Histogram::getMinChannelVal() {
	uint8_t maxVal=0xFF;
	for(uint8_t ch=0; ch < NUM_CHANNELS; ch++){
		if (maxVal > channels[ch]) maxVal = channels[ch];
	}
	return maxVal;
}

bool Histogram::updateHistogram(uint8_t ch) {

	uint8_t minVal = getMinChannelVal();
	for(uint8_t ch=0; ch < NUM_CHANNELS; ch++){
		channels[ch] -= minVal;
	}

	if (!buffer->hasCode()) return false;

	while(buffer->hasCode()){
		Code *code = buffer->getCode();
		channels[code->channel] = min(channels[code->channel] + 1, 255);
	}
	return true;
}

void Histogram::print() {
	printString("Histogram:\r\n");
	for (uint8_t ch=0;ch<NUM_CHANNELS; ch++){
		printString("CH:"); printByte(ch); printString(":");
		for(uint8_t i=0;i<channels[ch];i++){
			printString("#");
		}
		printString("\r\n");
	}
}



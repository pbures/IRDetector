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

#include "CmdBuffer.h"

class Histogram {

public:
	Histogram(CmdBuffer *buffer);
	bool updateHistogram();
	void print();

	uint8_t channels[NUM_CHANNELS];
private:
	void reset();
	uint8_t min(uint8_t a, uint8_t b);
	uint8_t max(uint8_t a, uint8_t b);
	uint8_t getMaxChannelVal();
	uint8_t getMinChannelVal();
	CmdBuffer* buffer;
};

#endif /* HISTOGRAM_H_ */

/*
 * LEDIndicators.cpp
 *
 *  Created on: 11. 11. 2016
 *      Author: pbures
 */

#include "LEDIndicators.h"

void LEDIndicators::init() {
	SET_OUTPUT_MODE(SR_DDR, SR_DTA);
	SET_OUTPUT_MODE(SR_DDR, SR_CLK);
	SET_OUTPUT_MODE(SR_DDR, SR_LAT);
	SET_OUTPUT_MODE(SR_DDR, SR_OE);
}

void LEDIndicators::setLeds(uint8_t leds) {

	SET_LOW(SR_PORT, SR_LAT);
	SET_LOW(SR_PORT, SR_CLK);
	SET_LOW(SR_PORT, SR_OE);

	for(uint8_t l=0;l<8;l++) {
		if (leds & (1<<l)) {
			SET_HIGH(SR_PORT, SR_DTA);
		} else {
			SET_LOW(SR_PORT, SR_DTA);
		}

		SET_HIGH(SR_PORT, SR_CLK);
		SET_LOW(SR_PORT, SR_CLK);
	}

	SET_HIGH(SR_PORT, SR_LAT);
}
// TODO Auto-generated constructor stub


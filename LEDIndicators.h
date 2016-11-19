/*
 * LEDIndicators.h
 *
 *  Created on: 11. 11. 2016
 *      Author: pbures
 */

#ifndef LEDINDICATORS_H_
#define LEDINDICATORS_H_

#define SR_PORT PORTC
#define SR_DDR  DDRC
#define SR_DTA  PC0 /* This is data signal */
#define SR_CLK  PC1 /* When value goes from low to high, the DTA is stored and values shifted */
#define SR_LAT  PC2 /* Keep low while writing to it */
#define SR_OE   PC3  /*Keep low for output enable */

#include <stdint.h>
#include <avr/io.h>
#include "config.h"
#include "pindefs.h"

/*
 * Drives up to 8 LEDs, enabling or disabling them.
 * This is a driver.
 * TODO: Make sure this is verified on a hardware!
 */
class LEDIndicators {
public:
	static void init();
	static void setLeds(uint8_t leds);
};

#endif /* LEDINDICATORS_H_ */

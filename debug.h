#ifndef DEBUG_H_
#define DEBUG_H_

#include "pindefs.h"

#ifdef DEBUG_STATE_MACHINE
#define printStringSMDebug(a) printf(a)
#else
#define printStringSMDebug(a)
#endif

#ifdef DEBUG_LEDS
enum DbgLed {
	RCVLED1,  //PB2 (pin 16)
	RCVLED2,  //PB0 (pin 14)
	RCVLED3,  //PB6 (pin  9)
	ALL,
	NONE
};
#endif

#ifdef DEBUG_LEDS
#define DBGLED(a,b) dbgLed(a,b)
#else
#define DBGLED(a,b)
#endif

void initDebugLeds(){
	DDRB |= ((1 << PB2) | (1 << PB0) | (1 << PB6));
}

#ifdef DEBUG_LEDS
void dbgLed(DbgLed led, uint8_t dir) {
	switch (led) {
	case NONE:
		break;

	case RCVLED1:
		if (dir)
			SET_HIGH(PORTB, PB0);
		else
			SET_LOW(PORTB, PB0);
		break;

	case RCVLED2:
		if (dir)
			SET_HIGH(PORTB, PB2);
		else
			SET_LOW(PORTB, PB2);
		break;

	case RCVLED3:
		if(dir)
			SET_HIGH(PORTB, PB6);
		else
			SET_HIGH(PORTB, PB6);
		break;

	case ALL:
		if (dir) {
			PORTB |= (1 << PB0) | (1 << PB2) | (1 << PB6);
		} else {
			PORTD &= ~((1 << PB2) | (1 << PB0) | (1 << PB6));
		}
	}
}
#endif

#endif //DEBUG_H_

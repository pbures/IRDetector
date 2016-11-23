/*
 * IRTransciever.cpp
 *
 *  Created on: 21. 11. 2016
 *      Author: pbures
 */

#include "IRTransciever.h"

IRTransciever::IRTransciever() {
	// TODO Auto-generated constructor stub
}

void IRTransciever::init() {
	/* Need to use different timer than timer0 */

	SET_OUTPUT_MODE(TRC_DDR, TRC_BIT);
	IROff();
	TCCR0A |= (1 << WGM01);  // CTC Mode
	TCCR0A |= (1 << COM0A0); // Toggle PD6 on cycle through
	TCCR0B |= (1 << CS00);   // Prescaler to 1
}

void IRTransciever::send(uint8_t byte) {
	initialBurst();

	for (uint8_t b = 7; b >= 0; b--) {
		IROn();
		delay560us();
		IROff();
		if ((1 << b) & byte)
			delay1200us();
		else
			delay560us();
	}

	/* Finish the last bit */
	IROn();
	delay560us();
	IROff();
}

void IRTransciever::IROn() {
	OCR0A = 105; // 8Mhz / 38 Khz. Blink every 210th tick, it is 105th up, 210th down. The time of one blink is 26us.
	TRC_DDR |= (1 << TRC_BIT);
}

void IRTransciever::IROff() {
	TRC_DDR &= ~(1 << TRC_BIT);
}

void IRTransciever::initialBurst() {
	IROn();
	_delay_ms(9);
	IROff();
	_delay_ms(4);

	/* Can not call directly _delay_us(500) as max allowed value is 768/ F_CPU (in Mhz). */
	for (uint8_t i = 0; i < 6; i++) {
		_delay_us(500 / 6);
	}
}
void inline IRTransciever::delay560us() {
	for (uint8_t i = 0; i < 6; i++)
		_delay_us(560 / 6);
}

void inline IRTransciever::delay1200us() {
	for (uint8_t i = 0; i < 13; i++)
		_delay_us(1200 / 13);
}

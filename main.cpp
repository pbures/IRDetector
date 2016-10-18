/*
 *  Created on: 11. 10. 2016
 *      Author: Pavel Bures
 */
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/power.h>

#include "USART/USART.h"

#define TRC_DDR 	DDRD
#define TRC_PORT 	PORTD
#define TRC_PIN 	PIND
#define TRC_BIT 	PD6

#define RCVLED1_DDR     DDRB
#define RCVLED1_PORT    PORTB
#define RCVLED1_PIN     PINB
#define RCVLED1_BIT     PB1

#define RCVLED2_DDR     DDRB
#define RCVLED2_PORT    PORTB
#define RCVLED2_PIN     PINB
#define RCVLED2_BIT     PB2

#define SET_INPUT_MODE(ddr,bit) ddr &= ~(1<<bit)
#define SET_OUTPUT_MODE(ddr,bit) ddr |= (1<<bit)
#define SET_HIGH(port,bit) port |= (1<<bit)
#define SET_LOW(port,bit) port &= ~(1<<bit)

int8_t volatile timerOverflowCnt[2] = { 0, 0 };
uint16_t volatile timerReg[2] = { 0, 0 };
volatile uint8_t irrcPins = 0;
volatile uint8_t started = 0;

#define TIMER_MAX 65535

enum TimeGap {
	OVER_BHIGH_GAP, OVER_BLOW_GAP, OVER_IMPULS_GAP, NONE
};

enum DbgLed {
	RED, GREEN, BLUE, YELLOW, RCVLED1, RCVLED2, GPDBG, ALL
};

enum Status {
	IDLE, BURST, RECV
};

Status volatile status[2] = { IDLE, IDLE };
uint8_t rcvPin[2] = { PC1, PC2 };

#define STORE_DATA
#define COMMANDS_BUFLEN 256
uint32_t volatile myByte[2] = { 0x00, 0x00 };
uint8_t volatile bitPtr[2] = { 0, 0 };
uint8_t volatile commands[2][COMMANDS_BUFLEN];
uint8_t volatile commandsPtr[2] = { 0, 0 };
uint8_t volatile commandsReadPtr[2] = { 0, 0 };

void printCommandsBuffer(uint8_t ch) {
	printString("Cmd buf ");
	printByte(ch);
	printString("=");
	while(commandsReadPtr[ch] != commandsPtr[ch]){
		printByte(commands[ch][commandsReadPtr[ch]]);
		printString(" ,");
		commandsReadPtr[ch] = commandsReadPtr[ch] + 1;
	}
	printString("\r");
}

/* TODO:
 * Make sure we are running on 8Mhz.
 */

/* TODO:
 * Need to implement the state machine, recognizing zero/one/space of NEC protocol:
 *
 * The NEC protocol uses pulse distance encoding of the bits. Each pulse is a 560µs
 * long 38kHz carrier burst (about 21 cycles). A logical "1" takes 2.25ms to transmit,
 * while a logical "0" is only half of that, being 1.125ms. The recommended carrier
 * duty-cycle is 1/4 or 1/3. *
 * Modulation is:
 * One : <|||560us|||______1.69ms_____>
 * Zero: <|||560us|||__560us__>
 *
 * A command is composed of:
 * <|||9ms|||_4.5ms_><<LSBaddr1><MSBaddr1>><<LSBaddr2><MSBaddr2>><<LSBcmd1....>
 */

DbgLed rcvHiCh[2] = { RED, GREEN };
DbgLed rcvOne[2] = { BLUE, YELLOW };
DbgLed rcvSig[2] = { RCVLED1, RCVLED2 };

void dbgLed(DbgLed led, uint8_t dir) {
	switch (led) {
	case GREEN:
		if (dir)
			SET_HIGH(PORTD, PD5);
		else
			SET_LOW(PORTD, PD5);
		break;

	case RED:
		if (dir)
			SET_HIGH(PORTD, PD7);
		else
			SET_LOW(PORTD, PD7);
		break;

	case BLUE:
		if (dir)
			SET_HIGH(PORTD, PD3);
		else
			SET_LOW(PORTD, PD3);
		break;

	case YELLOW:
		if (dir)
			SET_HIGH(PORTD, PD2);
		else
			SET_LOW(PORTD, PD2);
		break;

	case RCVLED1:
		if (dir)
			SET_HIGH(RCVLED1_PORT, RCVLED1_BIT);
		else
			SET_LOW(RCVLED1_PORT, RCVLED1_BIT);
		break;

	case RCVLED2:
		if (dir)
			SET_HIGH(RCVLED2_PORT, RCVLED2_BIT);
		else
			SET_LOW(RCVLED2_PORT, RCVLED2_BIT);
		break;

	case GPDBG:
		if (dir)
			SET_HIGH(PORTD, PD4);
		else
			SET_LOW(PORTD, PD4);
		break;

	case ALL:
		if (dir) {
			PORTD |= (1 << PD3) | (1 << PD5) | (1 << PD7);
			SET_HIGH(RCVLED1_PORT, RCVLED1_BIT);
			SET_HIGH(RCVLED2_PORT, RCVLED2_BIT);
		} else {
			PORTD &= ~((1 << PD3) | (1 << PD5) | (1 << PD7));
			SET_LOW(RCVLED1_PORT, RCVLED1_BIT);
			SET_LOW(RCVLED2_PORT, RCVLED2_BIT);
		}
	}
}

inline void initTimer() {
	TCNT1 = 0;
	TCCR1B |= (1 << CS10);
	TIMSK1 |= (1 << TOIE1);
}

inline void resetTime(uint8_t ch) {
	timerReg[ch] = TCNT1;
	timerOverflowCnt[ch] = 0;
	if (TIFR1 & (1 << TOV1)) {
		timerReg[ch] = TCNT1;
		timerOverflowCnt[ch] = -1;
	}
}

#define LO2HI(bit) ((!(irrcPins & (1 << bit))) &&  (pinc & (1 << bit)))
#define HI2LO(bit) (( (irrcPins & (1 << bit))) && !(pinc & (1 << bit)))
#define CHNG(bit) ((irrcPins ^ pinc) & (1<<bit))

#define NUM_CHANNELS 2
#define START_CHANNEL 0

ISR(PCINT1_vect) {

	uint8_t pinc = ~PINC;
	uint32_t timeUs[2] = { 0L, 0L };

	uint8_t processed[2] = { 0, 0 };

	/* TODO: Make sure we are not losing any pin change while we are in this ISR.
	 For now: go over all channels twice to check if some has changed during processing.
	 More intelligent is to observe the PCIFR register and check on PCIF1.
	 */
	for (uint8_t t = 0; t < 2; t++) {
		pinc = ~PINC;
		for (uint8_t ch = START_CHANNEL; ch < 2; ch++) {

			if (processed[ch])
				continue;

			if (!CHNG(rcvPin[ch]))
				continue;

			if (pinc & (1 << rcvPin[ch]))
				dbgLed(rcvSig[ch], 1);
			else
				dbgLed(rcvSig[ch], 0);

			/* Get the number of ticks, overflows on the counter and reset the counter. */
			uint16_t ticks = TCNT1;
			uint8_t overflows = timerOverflowCnt[ch];

			if (TIFR1 & (1 << TOV1)) { /* We have just encountered timer overflow. Overflows counters are not updated however. */
				ticks = TCNT1;
				overflows++;
			}

			/* Compute the time in microseconds */
			timeUs[ch] = ((overflows * 8192) + (ticks / 8) - (timerReg[ch] / 8));
			resetTime(ch);
		}

		for (uint8_t ch = START_CHANNEL; ch < NUM_CHANNELS; ch++) {

			if (processed[ch])
				continue;

			if (!CHNG(rcvPin[ch]))
				continue;

			processed[ch] = 1;
			dbgLed(GPDBG, 1);
			switch (status[ch]) {

			case IDLE:
				dbgLed(rcvHiCh[ch], 0);
				dbgLed(rcvOne[ch], 0);
				if (LO2HI(rcvPin[ch])) {
					dbgLed(rcvHiCh[ch], 0);
					status[ch] = BURST;
					dbgLed(GPDBG, 0);
				}
				break;

			case BURST:
				dbgLed(GPDBG, 0);
				if (HI2LO(rcvPin[ch])) {
					dbgLed(rcvHiCh[ch], 0);
					if (timeUs[ch] > 8900) {
						status[ch] = BURST;
					} else {
						status[ch] = IDLE;
					}
				} else {
					dbgLed(rcvOne[ch], 0);
					if (timeUs[ch] > 4400) {
						status[ch] = RECV;
						dbgLed(rcvHiCh[ch], 1);
					} else {
						status[ch] = BURST;
					}
				}
				break;

			case RECV:

				if (HI2LO(rcvPin[ch])) {
					dbgLed(rcvHiCh[ch], 0);
					dbgLed(rcvOne[ch], 0);

					if (timeUs[ch] > 9000) {
						status[ch] = BURST;
					} else { //Should be 560us
						status[ch] = RECV;
					}
				} else {
					if (timeUs[ch] > 4500) {
						status[ch] = BURST;
						dbgLed(GPDBG, 1);
					} else {
						status[ch] = RECV;
						dbgLed(rcvHiCh[ch], 1);
						if (timeUs[ch] > 660) { //Should be 560us
#ifdef STORE_DATA
							myByte[ch] |= (uint32_t) (((uint32_t) 1) << bitPtr[ch]);
#endif
							dbgLed(rcvOne[ch], 1);
						}
						bitPtr[ch]++;
					}
				}
				break;
			}

			if (bitPtr[ch] >= 32) {
#ifdef STORE_DATA
				commands[ch][commandsPtr[ch]] = (uint8_t) ((myByte[ch] >> 16) & 0xFF);
				commandsPtr[ch] = commandsPtr[ch] + 1;

				myByte[ch] = 0;
				bitPtr[ch] = 0;
				status[ch] = IDLE;
			}
			dbgLed(GPDBG, 0);
		}
	}
#endif
	irrcPins = ~PINC;
}

/* Count number of timer overflows, this tells us how many cycles passed */
ISR(TIMER1_OVF_vect) {
	if (timerOverflowCnt[0] < 127)
		timerOverflowCnt[0]++;
	if (timerOverflowCnt[1] < 127)
		timerOverflowCnt[1]++;
	TCNT1 = 0;
}

void IROn() {
	OCR0A = 105; // 8Mhz / 38 Khz. Blink every 210th tick, it is 105th up, 210th down. The time of one blink is 26us.
	TRC_DDR |= (1 << TRC_BIT);
}

void IROff() {
	TRC_DDR &= ~(1 << TRC_BIT);
}

void IRInit() {
	SET_OUTPUT_MODE(TRC_DDR, TRC_BIT);
	IROff();
	TCCR0A |= (1 << WGM01);  /* CTC Mode */
	TCCR0A |= (1 << COM0A0); /* Toggle PD6 on cycle through */
	TCCR0B |= (1 << CS00);   /* Prescaler to 1 */
}

/*
 * Initialize receivers on pins C, PC0, PC1, and PC2.
 */
void initReceivers() {
	SET_INPUT_MODE(DDRC, PC1);
	SET_HIGH(PORTC, PC1);

	PCICR |= (1 << PCIE1);
	PCMSK1 |= ((1 << PCINT9) | (1 << PCINT10));
	irrcPins = ~PINC;
}

int main() {

	SET_OUTPUT_MODE(RCVLED1_DDR, RCVLED1_BIT);
	SET_OUTPUT_MODE(RCVLED2_DDR, RCVLED2_BIT);
	DDRD |= ((1 << PD5) | (1 << PD7) | (1 << PD3) | (1 << PD2) | (1 << PD4));

	dbgLed(RED, 1);
	_delay_ms(800);
	dbgLed(RED, 0);
	dbgLed(GREEN, 1);
	_delay_ms(800);
	dbgLed(GREEN, 0);
	dbgLed(BLUE, 1);
	_delay_ms(800);
	dbgLed(ALL, 0);
	dbgLed(GPDBG, 0);

	initUSART();
	IRInit();

	initTimer();
	initReceivers();

	SET_OUTPUT_MODE(DDRB, PB0);
	sei();

	printString("Started!\r");

	while (true) {

		for (uint8_t ch = 0; ch < 2; ch++) {
			if (commandsReadPtr[ch] != commandsPtr[ch]) {
				printCommandsBuffer(ch);
			}
		}
		_delay_ms(5000);

//		IROn();
//		_delay_ms(2);
//		IROff();
//		_delay_ms(4);
	};
}


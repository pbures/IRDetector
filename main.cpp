/*
 *  Created on: 11. 10. 2016
 *      Author: Pavel Bures
 */
#include "config.h"

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <stdio.h>

#include "debug.h"
#include "pindefs.h"
#include "Code.h"
#include "Histogram.h"
#include "RingBuffer.h"
#include "LEDIndicators.h"


#include "uart.h"


#if F_CPU != 8000000
#error "Please run the atmega chip at 8Mhz to reach the right timing."
#endif

enum TimeGap {
	G9P0MS = 3, G4P5MS = 2, G1P2MS = 1, G0P5MS = 0
};

enum Status {
	IDLE, BURST, RECV
};

/* Timer variables */
#define TIMER_MAX 256 //65535
int8_t volatile timerOverflowCnt;
uint16_t volatile timerReg;
volatile uint8_t started = 0;

/* Buffer for BANK C changes */
volatile uint8_t portStatus[NUM_READINGS];
volatile uint8_t portStatusTime[NUM_READINGS];
volatile uint8_t portStatusPtr = 0;
volatile uint8_t portStatusReadPtr = 0;

/* State machine variables */
uint8_t channelStatusLast;
uint8_t channelStatusTimeGap[8];
Status channelSMStatus[8];
uint16_t myWord[NUM_CHANNELS];
uint8_t bitPtr[NUM_CHANNELS];

RingBuffer<Code,64> ringBuffer;
RingBufferIterator<Code,64> ringBufferIterator(&ringBuffer);
Histogram<Code,64> hist(&ringBuffer);

/*
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


static int uart_putchar(char c, FILE *stream);
static FILE mystdout = {0};


// http://www.nongnu.org/avr-libc/user-manual/group__avr__stdio.html
static int uart_putchar(char c, FILE *stream) {
	txByte(c);
	return 0;
};

inline void resetTime() {
	timerReg = TCNT0;
	timerOverflowCnt = 0;
	if (TIFR0 & (1 << TOV0)) {
		timerReg = TCNT0;
		timerOverflowCnt = -1;
	}
}


ISR(PCINT2_vect) {

	uint8_t pind = ~PIND;
#ifdef DEBUG_LEDS
	DBGLED(RCVLED1, 1);
#endif

	/* Get the number of ticks, overflows on the counter and reset the counter. */
	uint16_t ticks = TCNT0;
	uint8_t overflows = timerOverflowCnt;

	if (TIFR0 & (1 << TOV0)) { /* We have just encountered timer overflow. Overflows counters are not updated however. */
		ticks = TCNT0;
		overflows++;
	}

	/* Compute the time in units of 64 microseconds. We are running 8MhZ and clock prescaler is set to 256, and divide by two
	 so we fit into uint16_t in case we have overflows equal to 1. 64us is the timeUs resolution. */
	uint16_t timeUs = ((overflows * TIMER_MAX / 2) + (ticks >> 1)
			- (timerReg >> 1));
	resetTime();

	if (pind != portStatus[portStatusPtr]) {
		if (timeUs > 0) { /* This is another event within 64us, store into the current status */
			portStatusPtr = (portStatusPtr + 1) % NUM_READINGS;
			portStatusTime[portStatusPtr] = (timeUs > 255) ? 255 : timeUs;
		}
		portStatus[portStatusPtr] = pind;
	}

#ifdef DEBUG_LEDS
	DBGLED(RCVLED1, 0);
#endif
}

/* Count number of timer overflows, this tells us how many cycles passed */
ISR(TIMER0_OVF_vect) {
	/* We are fine to count up to two overflows. Moreover, if we keep it this low, the numner of
	 * microseconds can be stored in uint16_t which saves us some time in ISR.
	 */
	if (timerOverflowCnt < 254)
		timerOverflowCnt++;
	TCNT0 = 0;
}

/*
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
 TCCR0A |= (1 << WGM01); // CTC Mode
 TCCR0A |= (1 << COM0A0); // Toggle PD6 on cycle through
 TCCR0B |= (1 << CS00); // Prescaler to 1
 }
 */

/*
 * Initialize receivers on pins D, PD2, PD3, ....
 */
void initReceivers() {

	DDRD &= ~((1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5) | (1 << PD6)
			| (1 << PD7));
	PORTD |= ((1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5) | (1 << PD6)
			| (1 << PD7));

	/* Enable the pin change interrupts on these three pins corresponding to PD2, PD3, ... */
	PCICR |= (1 << PCIE2);
	PCMSK2 |= ((1 << PCINT18) | (1 << PCINT19) | (1 << PCINT20) | (1 << PCINT21)
			| (1 << PCINT22) | (1 << PCINT23));

	/* Store the actual status of the PIND port so we can see what has changed in ISR */
	channelStatusLast = ~PIND;
}

void initBuffers() {
	for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
		myWord[ch] = 0;
		bitPtr[ch] = 0;
		channelStatusTimeGap[ch] = 0;
		channelSMStatus[ch] = IDLE;
	}

}

inline void initTimer() {
	TCNT0 = 0;
	TCCR0A = 0;
	TCCR0B = ((1 << CS02) /* | (1 << CS00) | (1 << CS01) */); /* Prescaler set to 256 */
	TIMSK0 = (1 << TOIE0);

	timerOverflowCnt = 0;
	timerReg = 0;
}

void initServo() {
	TCCR1A |= (1 << WGM11);
	TCCR1B |= ((1 << WGM12) | (1 << WGM13));

	TCCR1B |= (1 << CS11); //Prescaler to 8MhZ, each tick = 1us.
	TCCR1A |= (1 << COM1A1);

	ICR1 = 20000; //20ms cycle.
	DDRB |= (1 << PB1);

	OCR1A = 1500;

	DDRB |= (1 << PB1);

}

void processPortStateChange(uint8_t ptr) {

	uint8_t portState = portStatus[ptr];

	for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
		uint16_t gap = channelStatusTimeGap[ch] + portStatusTime[ptr];
		channelStatusTimeGap[ch] = (gap > 255) ? 255 : gap;
	}

	for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) { /* Let us start with single channel */
		uint8_t lo2hi = 0;
		printStringSMDebug("\r\nch:[");printByteSMDebug(ch);printStringSMDebug("]");
		if ((portState ^ channelStatusLast) & (1 << (ch + FIRST_CHANNEL))) {

			if (portState & (1 << (ch + FIRST_CHANNEL))) {
				lo2hi = 1;
			}

			/* If there was a change on the given channel */
			if (lo2hi) {
				printStringSMDebug("LOHI");
			} else {
				printStringSMDebug("HILO");
			}

			TimeGap timeGap;
			uint8_t gap = channelStatusTimeGap[ch];
			printStringSMDebug(" In: ");

			if (gap < 900 / 64) {
				timeGap = G0P5MS;
				printStringSMDebug(" G0P5MS");
			} else if (gap < 4400 / 64) {
				timeGap = G1P2MS;
				printStringSMDebug(" G1P2MS");
			} else if (gap < 8900 / 64) {
				timeGap = G4P5MS;
				printStringSMDebug(" G4P5MS");
			} else {
				timeGap = G9P0MS;
				printStringSMDebug(" G9P0MS");
			}

			printStringSMDebug(" (");printByteSMDebug(gap);printStringSMDebug(")");

			switch (channelSMStatus[ch]) {
			case IDLE:
				if (lo2hi) {
					channelSMStatus[ch] = BURST;
					printStringSMDebug(" IDLE->BURST");
				}
				break;

			case BURST:
				bitPtr[ch] = 0;
				myWord[ch] = 0;
				if (!lo2hi) {
					if (timeGap == G9P0MS) {
						channelSMStatus[ch] = BURST;
						printStringSMDebug(" BURST->BURST");
					} else {
						channelSMStatus[ch] = IDLE;
						printStringSMDebug(" BURST->IDLE");
					}
				} else {
					if (timeGap >= G4P5MS) {
						channelSMStatus[ch] = RECV;
						printStringSMDebug(" BURST->RECV");
					} else {
						channelSMStatus[ch] = BURST;
						printStringSMDebug(" BURST->BURST");
					}
				}
				break;

			case RECV:

				if (!lo2hi) {
					if (timeGap == G9P0MS) {
						channelSMStatus[ch] = BURST;
						printStringSMDebug(" RECV->BURST");
					} else {
						channelSMStatus[ch] = RECV;
						printStringSMDebug(" RECV->RECV");
					}
				} else {
					if (timeGap >= G4P5MS) {
						channelSMStatus[ch] = BURST;
						printStringSMDebug(" RECV->BURST");
					} else {
						channelSMStatus[ch] = RECV;
						printStringSMDebug(" RECV->RECV");printStringSMDebug(" b:");printByteSMDebug(bitPtr[ch]);

						if (timeGap >= G1P2MS) {
#ifdef STORE_DATA
							if (bitPtr[ch] > 15)
								myWord[ch] |= (uint16_t) ((uint16_t) 1
										<< (bitPtr[ch] - 16));
#endif
							printStringSMDebug(" ONE");
						} else {
							printStringSMDebug(" ZERO");
						}
#ifdef STORE_DATA
						bitPtr[ch]++;
#endif
					}
				}
				break;
			}

#ifdef STORE_DATA
			if (bitPtr[ch] >= 32) {
#ifdef STORE_DATA
				uint8_t cmd = (uint8_t) ((myWord[ch]) & 0xFF);
				uint8_t cmdNeg = (uint8_t) ((myWord[ch] >> 8) & 0xFF);
				Code c;
				if ((cmd ^ cmdNeg) == 0xFF) {
					printStringSMDebug(" ST");printStringSMDebug("(");printByteSMDebug(ch);printStringSMDebug("): ");printBinaryByteSMDebug(cmd);printStringSMDebug("\r\n");

					c.channel = ch;
					c.code = cmd;
					ringBuffer.add(&c);
				}

#endif
				myWord[ch] = 0;
				bitPtr[ch] = 0;
				channelSMStatus[ch] = IDLE;
			}
#endif

			channelStatusTimeGap[ch] = 0;
		} else {
			printStringSMDebug("NO CHNG.");
		}

	}
	channelStatusLast = portState;
}

void processPortStateChanges() {
	while (portStatusReadPtr != portStatusPtr) {
		processPortStateChange(portStatusReadPtr);

		//printString("\r\n");printBinaryByte(portStatus[portStatusReadPtr]);printString(" dt:");printByte(portStatusTime[portStatusReadPtr]);

		portStatusReadPtr = (portStatusReadPtr + 1) % NUM_READINGS;
	}
}

int main() {

#ifdef DEBUG_LEDS
	initDebugLeds();
	DBGLED(RCVLED1, 1);
	_delay_ms(500);
	DBGLED(RCVLED1, 0);
	DBGLED(RCVLED2, 1);
	_delay_ms(500);
	DBGLED(RCVLED2, 0);
	DBGLED(RCVLED3, 1);
	_delay_ms(500);
	DBGLED(RCVLED3, 0);
	DBGLED(ALL, 0);
#endif

	initUart();

	fdev_setup_stream(&mystdout, uart_putchar, NULL, _FDEV_SETUP_WRITE);
	stdout = &mystdout;

	/*
	 IRInit();
	 */
	initTimer();
	initReceivers();
	initBuffers();
	initServo();
	LEDIndicators::init();

	for (uint8_t i=0; i< (3*6); i++) {
		LEDIndicators::setLeds(2<<(i % 6));
		_delay_ms(50);
		LEDIndicators::setLeds(0);
		_delay_ms(50);
	}

	LEDIndicators::setLeds(0);

	sei();
	printf("Started!\r\n");

	while (true) {
		processPortStateChanges();
		if (hist.updateHistogram()) {
			hist.print();
			LEDIndicators::setLeds(hist.getMainChannels() << 1);
		}

		ringBufferIterator.print();
		ringBuffer.reset();
	}
}




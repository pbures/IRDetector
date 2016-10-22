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

#define RCVLED3_DDR     DDRB
#define RCVLED3_PORT    PORTB
#define RCVLED3_PIN     PINB
#define RCVLED3_BIT     PB0

#define START_CHANNEL 0
#define NUM_CHANNELS 3

#define SET_INPUT_MODE(ddr,bit) ddr &= ~(1<<bit)
#define SET_OUTPUT_MODE(ddr,bit) ddr |= (1<<bit)
#define SET_HIGH(port,bit) port |= (1<<bit)
#define SET_LOW(port,bit) port &= ~(1<<bit)

int8_t volatile timerOverflowCnt[3] = { 0, 0, 0 };
uint16_t volatile timerReg[3] = { 0, 0, 0 };
volatile uint8_t irrcPins = 0;
volatile uint8_t started = 0;

#define TIMER_MAX 65535

enum TimeGap {
	G9P0MS = 5, G4P5MS = 4, G1P2MS = 3, G0P5MS = 2
};

enum DbgLed {
	GREEN,    //PD5 (pin 11)
	ISRDBG,   //PD7 (pin 13)
	RCVLED1,  //PB1 (pin 15) (CH0)
	RCVLED2,  //PB2 (pin 16) (CH1)
	RCVLED3,  //PB0 (pin 14) (CH2)
	GPDBG1,   //PD2 (pin  4)
	GPDBG2,   //PD3 (pin  5)
	GPDBG3,   //PD4 (pin  6)
	ALL, NONE
};

enum Status {
	IDLE, BURST, RECV
};

Status volatile status[3] = { IDLE, IDLE, IDLE };
uint8_t rcvPin[3] = { PC1, PC2, PC3 };

#define STORE_DATA
#define COMMANDS_BUFLEN 256
uint32_t volatile myByte[3] = { 0x00, 0x00, 0x00 };
uint8_t volatile bitPtr[3] = { 0, 0, 0 };
uint8_t volatile commands[3][COMMANDS_BUFLEN];
uint8_t volatile commandsPtr[3] = { 0, 0, 0 };
uint8_t volatile commandsReadPtr[3] = { 0, 0, 0 };

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

DbgLed rcvHiCh[3] = { NONE, NONE, NONE };
DbgLed rcvOne[3] = { GPDBG1, GPDBG2, GPDBG3 };
DbgLed rcvSig[3] = { RCVLED1, RCVLED2, RCVLED3 };

void dbgLed(DbgLed led, uint8_t dir) {
	switch (led) {
	case NONE:
		break;
	case GREEN:
		if (dir)
			SET_HIGH(PORTD, PD5);
		else
			SET_LOW(PORTD, PD5);
		break;

	case ISRDBG:
		if (dir)
			SET_HIGH(PORTD, PD7);
		else
			SET_LOW(PORTD, PD7);
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

	case RCVLED3:
		if (dir)
			SET_HIGH(RCVLED3_PORT, RCVLED3_BIT);
		else
			SET_LOW(RCVLED3_PORT, RCVLED3_BIT);
		break;

	case GPDBG1:
		if (dir)
			SET_HIGH(PORTD, PD2);
		else
			SET_LOW(PORTD, PD2);
		break;

	case GPDBG2:
		if (dir)
			SET_HIGH(PORTD, PD3);
		else
			SET_LOW(PORTD, PD3);
		break;

	case GPDBG3:
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


ISR(PCINT1_vect) {

	uint8_t pinc;
	uint8_t processed = 0;
	TimeGap timeGap[3];

	dbgLed(ISRDBG,1);
	do {
		pinc = ~PINC;
		if (PCIFR & (1<<PCIF1)) {
			pinc = ~PINC;
			PCIFR |= (1<<PCIF1);
		}

		for (uint8_t ch = START_CHANNEL; ch < NUM_CHANNELS; ch++) {
			if (processed & (1<<ch))
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

			/* Compute the time in microseconds, divide the ticks by 8 (>>3) as we are on 8MhZ */
			uint16_t timeUs = ((overflows * 8192) + (ticks >> 3) - (timerReg[ch] >> 3));
			resetTime(ch);

			if (timeUs < 900) {
				timeGap[ch] = G0P5MS;
			} else if (timeUs < 4400) {
				timeGap[ch] = G1P2MS;
			} else if (timeUs < 8900) {
				timeGap[ch] = G4P5MS;
			} else {
				timeGap[ch] = G9P0MS;
			}
		}

		for (uint8_t ch = START_CHANNEL; ch < NUM_CHANNELS; ch++) {

			if (processed & (1<<ch))
				continue;

			if (!CHNG(rcvPin[ch]))
				continue;

			processed |= (1<<ch);
			dbgLed(rcvOne[ch], 0);

			switch (status[ch]) {
			case IDLE:
				if (LO2HI(rcvPin[ch])) {
					status[ch] = BURST;
				}
				break;

			case BURST:
				if (HI2LO(rcvPin[ch])) {
					if (timeGap[ch] == G9P0MS) {//(timeUs[ch] > 9000) {
						status[ch] = BURST;
					} else {
						status[ch] = IDLE;
					}
				} else {
					if (timeGap[ch] >= G4P5MS) {//(timeUs[ch] > 4400) {
						status[ch] = RECV;
					} else {
						status[ch] = BURST;
					}
				}
				break;

			case RECV:

				if (HI2LO(rcvPin[ch])) {
					if (timeGap[ch] == G9P0MS) {//(timeUs[ch] > 9000) {
						status[ch] = BURST;
					} else { //Should be 560us
						status[ch] = RECV;
					}
				} else {
					if (timeGap[ch] >= G4P5MS) {//(timeUs[ch] > 4500) {
						status[ch] = BURST;
					} else {
						status[ch] = RECV;
						if (timeGap[ch] >= G1P2MS){//(timeUs[ch] > 900) {
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
				uint8_t cmd = (uint8_t) ((myByte[ch] >> 16) & 0xFF);
				uint8_t cmdNeg = (uint8_t) ((myByte[ch] >> 24) & 0xFF);

				if ((cmd ^ cmdNeg) == 0xFF) {
					commands[ch][commandsPtr[ch]] = cmd;
					commandsPtr[ch] = commandsPtr[ch] + 1;
				}

#endif
				myByte[ch] = 0;
				bitPtr[ch] = 0;
				status[ch] = IDLE;
			}
		}
	} while (PCIFR & (1<<PCIF1));

	irrcPins = ~PINC;
	dbgLed(ISRDBG,0);
}

/* Count number of timer overflows, this tells us how many cycles passed */
ISR(TIMER1_OVF_vect) {
	for(uint8_t ch=0;ch<NUM_CHANNELS;ch++){
		if(timerOverflowCnt[ch] < 3) timerOverflowCnt[ch]++;
	}
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
	SET_INPUT_MODE(DDRC, PC2);
	SET_HIGH(PORTC, PC2);
	SET_INPUT_MODE(DDRC, PC3);
	SET_HIGH(PORTC, PC3);

	PCICR |= (1 << PCIE1);
	PCMSK1 |= ((1 << PCINT9) | (1 << PCINT10) | ( 1<< PCINT11));
	irrcPins = ~PINC;
}

int main() {

	SET_OUTPUT_MODE(RCVLED1_DDR, RCVLED1_BIT);
	SET_OUTPUT_MODE(RCVLED2_DDR, RCVLED2_BIT);
	SET_OUTPUT_MODE(RCVLED3_DDR, RCVLED3_BIT);

	DDRD |= ((1 << PD5) | (1 << PD7) | (1 << PD3) | (1 << PD2) | (1 << PD4));

	dbgLed(RCVLED1, 1);
	_delay_ms(800);
	dbgLed(RCVLED1, 0);
	dbgLed(RCVLED2, 1);
	_delay_ms(800);
	dbgLed(RCVLED2, 0);
	dbgLed(RCVLED3, 1);
	_delay_ms(800);
	dbgLed(RCVLED3, 0);
	dbgLed(ALL, 0);

	initUSART();
	IRInit();

	initTimer();
	initReceivers();

	sei();

	printString("Started!\r");

	while (true) {

		for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
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


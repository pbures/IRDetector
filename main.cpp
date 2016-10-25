/*
 *  Created on: 11. 10. 2016
 *      Author: Pavel Bures
 */
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/power.h>

#include "USART/USART.h"

#if F_CPU != 8000000
#error "Please run the atmega chip at 8Mhz to reach the right timing."
#endif

#define DEBUG
#define DECODE_IN_MAIN
#define STORE_DATA
#define NUM_CHANNELS 6

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

#define SET_INPUT_MODE(ddr,bit) ddr &= ~(1<<bit)
#define SET_OUTPUT_MODE(ddr,bit) ddr |= (1<<bit)
#define SET_HIGH(port,bit) port |= (1<<bit)
#define SET_LOW(port,bit) port &= ~(1<<bit)

int8_t volatile timerOverflowCnt[NUM_CHANNELS];
uint16_t volatile timerReg[NUM_CHANNELS];
volatile uint8_t irrcPins = 0;
volatile uint8_t started = 0;

/* Encode here: Gap length,edge type (raising == MSB set to 1, falling == MSB set to 0) */
#ifdef DECODE_IN_MAIN
#define GAPS_BUFFER_LEN 256

volatile uint8_t gaps[NUM_CHANNELS][GAPS_BUFFER_LEN];
volatile uint8_t gapsPtr[NUM_CHANNELS];
volatile uint8_t gapsReadPtr[NUM_CHANNELS];
#endif

#define TIMER_MAX 65535

enum TimeGap {
	G9P0MS = 3, G4P5MS = 2, G1P2MS = 1, G0P5MS = 0
};

enum Status {
	IDLE, BURST, RECV
};

#ifdef DEBUG
enum DbgLed {
	ISRDBG2,  //PD5 (pin 11)
	ISRDBG1,  //PD7 (pin 13)
	RCVLED1,  //PB1 (pin 15) (CH0)
	RCVLED2,  //PB2 (pin 16) (CH1)
	RCVLED3,  //PB0 (pin 14) (CH2)
	GPDBG1,   //PD2 (pin  4)
	GPDBG2,   //PD3 (pin  5)
	GPDBG3,   //PD4 (pin  6)
	ALL, NONE
};
#endif

#if NUM_CHANNELS != 6
#error "Adjust the arrays below for proper number of channels!"
#endif

#ifdef DEBUG
DbgLed rcvHiCh[NUM_CHANNELS] = { NONE, NONE, NONE, NONE, NONE,NONE };
DbgLed rcvOne[NUM_CHANNELS] = { GPDBG1, GPDBG2, GPDBG3, NONE, NONE,NONE };
DbgLed rcvSig[NUM_CHANNELS] = { RCVLED1, RCVLED2, RCVLED3, NONE, NONE, NONE };
#endif

Status volatile status[NUM_CHANNELS] = { IDLE, IDLE, IDLE, IDLE, IDLE, IDLE };
uint8_t rcvPin[NUM_CHANNELS] = { PC0, PC1, PC2, PC3, PC4, PC5 };

#define COMMANDS_BUFLEN 16
uint16_t volatile myByte[NUM_CHANNELS];
uint8_t volatile bitPtr[NUM_CHANNELS];
uint8_t volatile commands[NUM_CHANNELS][COMMANDS_BUFLEN];
uint8_t volatile commandsPtr[NUM_CHANNELS];
uint8_t volatile commandsReadPtr[NUM_CHANNELS];

void printCommandsBuffer(uint8_t ch) {
	printString("Cmd buf ");
	printByte(ch);
	printString("=");
	while(commandsReadPtr[ch] != commandsPtr[ch]){
		printByte(commands[ch][commandsReadPtr[ch]]);
		printString(" ,");
		commandsReadPtr[ch] = (commandsReadPtr[ch] + 1) % COMMANDS_BUFLEN;
	}
	printString("\r");
}

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
#ifdef DEBUG
void dbgLed(DbgLed led, uint8_t dir) {
	switch (led) {
	case NONE:
		break;
	case ISRDBG2:
		if (dir)
			SET_HIGH(PORTD, PD5);
		else
			SET_LOW(PORTD, PD5);
		break;

	case ISRDBG1:
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
#endif

inline void initTimer() {
	TCNT1 = 0;
	TCCR1B |= ((1 << CS12) /* | (1 << CS10) | (1 << CS11) */);
	TIMSK1 |= (1 << TOIE1);

	for(uint8_t ch=0; ch < NUM_CHANNELS; ch++) {
		timerOverflowCnt[ch] = 0;
		timerReg[ch] = 0;
	}
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
/*
 * lo2hi ... 1 if rising edge, 0 otherwise
 */
inline void processEdge(uint8_t ch, uint8_t lo2hi, TimeGap timeGap) {
#ifdef DEBUG
	dbgLed(rcvOne[ch], 0);
#endif
	switch (status[ch]) {
	case IDLE:
		if (lo2hi) {
			status[ch] = BURST;
		}
		break;

	case BURST:
		if (!lo2hi) {
			if (timeGap == G9P0MS) {//(timeUs[ch] > 9000) {
				status[ch] = BURST;
			} else {
				status[ch] = IDLE;
			}
		} else {
			if (timeGap >= G4P5MS) {//(timeUs[ch] > 4400) {
				status[ch] = RECV;
			} else {
				status[ch] = BURST;
			}
		}
		break;

	case RECV:

		if (!lo2hi) {
			if (timeGap == G9P0MS) {//(timeUs[ch] > 9000) {
				status[ch] = BURST;
			} else { //Should be 560us
				status[ch] = RECV;
			}
		} else {
			if (timeGap >= G4P5MS) {//(timeUs[ch] > 4500) {
				status[ch] = BURST;
			} else {
				status[ch] = RECV;
				if (timeGap >= G1P2MS){//(timeUs[ch] > 900) {
#ifdef STORE_DATA
					if (bitPtr[ch] > 15)
					myByte[ch] |= (uint16_t) ((uint16_t) 1 << (bitPtr[ch]-16));
#endif
#ifdef DEBUG
					dbgLed(rcvOne[ch], 1);
#endif
				}
				bitPtr[ch]++;
			}
		}
		break;
	}

	if (bitPtr[ch] >= 32) {
#ifdef STORE_DATA
		uint8_t cmd = (uint8_t) ((myByte[ch]) & 0xFF);
		uint8_t cmdNeg = (uint8_t) ((myByte[ch] >> 8) & 0xFF);

		if ((cmd ^ cmdNeg) == 0xFF) {
			commands[ch][commandsPtr[ch]] = cmd;
			commandsPtr[ch] = (commandsPtr[ch] + 1) % COMMANDS_BUFLEN;
		}

#endif
		myByte[ch] = 0;
		bitPtr[ch] = 0;
		status[ch] = IDLE;
	}
}

ISR(PCINT1_vect) {

	/* The ISR takes in general about 60us for each channel. We are reaching the limit,
	 * and the status evaluation will have to be done outside the ISR.
	 */

	uint8_t pinc;
	uint8_t processed = 0;
	TimeGap timeGap[NUM_CHANNELS];
#ifdef DEBUG
	dbgLed(ISRDBG1,1);
#endif
	do {
		pinc = ~PINC;
		if (PCIFR & (1<<PCIF1)) {
			pinc = ~PINC;
			PCIFR |= (1<<PCIF1);
		}

		for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
			if (processed & (1<<ch))
				continue;

			if (!CHNG(rcvPin[ch]))
				continue;
#ifdef DEBUG
			if (pinc & (1 << rcvPin[ch]))
				dbgLed(rcvSig[ch], 1);
			else
				dbgLed(rcvSig[ch], 0);
#endif
			/* Get the number of ticks, overflows on the counter and reset the counter. */
			uint16_t ticks = TCNT1;
			uint8_t overflows = timerOverflowCnt[ch];

			if (TIFR1 & (1 << TOV1)) { /* We have just encountered timer overflow. Overflows counters are not updated however. */
				ticks = TCNT1;
				overflows++;
			}

			/* Compute the time in units of 64 microseconds. We are running 8MhZ and clock prescaler is set to 256, and divide by two
			   so we fit into uint16_t in case we have overflows equal to 1. */
			uint32_t timeUs = ((overflows * TIMER_MAX/2) + (ticks >> 1) - (timerReg[ch] >> 1));
			resetTime(ch);

			if (timeUs < 900/64) {
				timeGap[ch] = G0P5MS;
			} else if (timeUs < 4400/64) {
				timeGap[ch] = G1P2MS;
			} else if (timeUs < 8900/64) {
				timeGap[ch] = G4P5MS;
			} else {
				timeGap[ch] = G9P0MS;
			}

#ifdef DECODE_IN_MAIN
			if (HI2LO(rcvPin[ch])){
				gaps[ch][gapsPtr[ch]] = timeGap[ch];
			} else {
				gaps[ch][gapsPtr[ch]] = (timeGap[ch] | (1 << 7));
			}
			gapsPtr[ch] = (gapsPtr[ch] + 1) % GAPS_BUFFER_LEN;

			processed |= (1<<ch);
#endif
		}

#ifndef DECODE_IN_MAIN
		for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {

			if (processed & (1<<ch))
				continue;

			if (!CHNG(rcvPin[ch]))
				continue;

			processed |= (1<<ch);
			dbgLed(ISRDBG2,1);
			uint8_t dir  = (LO2HI(rcvPin[ch])) ? 1 : 0;
			processEdge(ch, dir,timeGap[ch]);
			dbgLed(ISRDBG2,0);
		}
#endif //DECODE_IN_MAIN

	} while (PCIFR & (1<<PCIF1));

	irrcPins = ~PINC;
#ifdef DEBUG
	dbgLed(ISRDBG1,0);
#endif
}

/* Count number of timer overflows, this tells us how many cycles passed */
ISR(TIMER1_OVF_vect) {
	for(uint8_t ch=0;ch<NUM_CHANNELS;ch++){
		/* We are fine to count up to two overflows. Moreover, if we keep it this low, the numner of
		 * microseconds can be stored in uint16_t which saves us some time in ISR.
		 */
		if(timerOverflowCnt[ch] == 0) timerOverflowCnt[ch]++;
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

	DDRC &= ~( (1<<PC1) | (1<<PC2) | (1<<PC3) | (1<<PC4) | (1<<PC5) | (1<<PC0));
	PORTC |= ( (1<<PC1) | (1<<PC2) | (1<<PC3) | (1<<PC4) | (1<<PC5) | (1<<PC0));

	/* Enable the pin change interrupts on these three pins corresponding to PC1,PC2,PC3 */
	PCICR |= (1 << PCIE1);
	PCMSK1 |= ((1<<PCINT8) | (1 << PCINT9) | (1 << PCINT10) | (1 << PCINT11) | (1 << PCINT12)
			| (1 << PCINT13));

	/* Store the actual status of the PINC port so we can see what has changed in ISR */
	irrcPins = ~PINC;
}

void initBuffers(){
	for (uint8_t ch=0;ch<NUM_CHANNELS; ch++){
		myByte[ch] = 0;
		bitPtr[ch] = 0;
		commandsPtr[ch] = 0;
		commandsReadPtr[ch] = 0;
#ifdef DECODE_IN_MAIN
		gapsPtr[ch] = 0;
		gapsReadPtr[ch] = 0;
#endif
	}
}

int main() {

	SET_OUTPUT_MODE(RCVLED1_DDR, RCVLED1_BIT);
	SET_OUTPUT_MODE(RCVLED2_DDR, RCVLED2_BIT);
	SET_OUTPUT_MODE(RCVLED3_DDR, RCVLED3_BIT);

	DDRD |= ((1 << PD5) | (1 << PD7) | (1 << PD3) | (1 << PD2) | (1 << PD4));
#ifdef DEBUG
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
#endif
	initUSART();
	IRInit();

	initTimer();
	initReceivers();
	initBuffers();

	sei();

	printString("Started!\r");

	while (true) {

#ifdef DECODE_IN_MAIN
		for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
			while (gapsReadPtr[ch] != gapsPtr[ch]){
				uint8_t gap = gaps[ch][gapsReadPtr[ch]];
				uint8_t dir = (gap & (1<<7)) ? 1 : 0;
				processEdge(ch, dir, (TimeGap) (gap & 0b01111111));
				gapsReadPtr[ch] = (gapsReadPtr[ch] + 1) % GAPS_BUFFER_LEN;
			}
		}
#endif
		for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
			if (commandsReadPtr[ch] != commandsPtr[ch]) {
				printCommandsBuffer(ch);
			}
		}

		/* Just to simulate the CPU does sth else */
		_delay_ms(1000);

//		IROn();
//		_delay_ms(2);
//		IROff();
//		_delay_ms(4);
	};
}


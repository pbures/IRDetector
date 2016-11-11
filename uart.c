#include <avr/io.h>
#include "uart.h"
#include <util/setbaud.h>
#include <avr/sfr_defs.h>

/* Define UART_SIMPLE for blocking, non-interrupt based implementation. */
// #define UART_SIMPLE

#ifndef UART_SIMPLE

/* Code for interrupt-based UART handling. */
#include <avr/interrupt.h>

/* UART buffer sizes (used for interrupt based (Async) Rx and Tx)).
* Has to be 2,4,8,16,32,64,128 or 256 bytes. */
#ifndef UART_RX_BUFFER_SIZE
#define UART_RX_BUFFER_SIZE 8
#endif

#ifndef UART_TX_BUFFER_SIZE
#define UART_TX_BUFFER_SIZE 8
#endif

/* UART buffer size masks */
#define UART_RX_BUFFER_MASK (UART_RX_BUFFER_SIZE - 1)
#if (UART_RX_BUFFER_SIZE & UART_RX_BUFFER_MASK)
#error RX buffer size is not a power of 2
#endif

#define UART_TX_BUFFER_MASK (UART_TX_BUFFER_SIZE - 1)
#if (UART_TX_BUFFER_SIZE & UART_TX_BUFFER_MASK)
#error TX buffer size is not a power of 2
#endif

/* Rx and Tx buffers */
static uint8_t rxBuffer[UART_RX_BUFFER_SIZE];
static volatile uint8_t rxHead;
static volatile uint8_t rxTail;
static uint8_t txBuffer[UART_TX_BUFFER_SIZE];
static volatile uint8_t txHead;
static volatile uint8_t txTail;

#endif /* UART_SIMPLE */

/* Initialization, needs defined F_CPU and BAUD */
void initUart(void) {

	/* prescaler (uses setbaud.h) */
	UBRR0H = UBRRH_VALUE;
	UBRR0L = UBRRL_VALUE;
	#if USE_2X
	UCSR0A |= (1 << U2X0);
	#else
	UCSR0A &= ~(1 << U2X0);
	#endif

	/* enable UART receiver (RXEN0) and transmitter (TXEN0) */
	UCSR0B = (1<<RXEN0) | (1<<TXEN0);
	/* asynchronous, no parity bit, 1 stop bit, 8 data bits, a.k.a. 8-N-1 */
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);

	/* additional interrupt-based init */
	#ifndef UART_SIMPLE

	/* enable  RX complete interrupt (RXCIE0) */
	UCSR0B |= (1<<RXCIE0);

	/* initialize buffers */
	rxTail = 0;
	rxHead = 0;
	txTail = 0;
	txHead = 0;

	/* enable interrupts */
	sei();
	#endif /* UART_SIMPLE */
}

#ifdef UART_SIMPLE

/* Test is there are unread data in the Rx buffer. */
uint8_t isRxAvailable() {
	return bit_is_set(UCSR0A, RXC0);
}

/* Test if Tx is ready to receive new data. */
uint8_t isTxAvailable() {
	return bit_is_set(UCSR0A, UDRE0);
}

/* Simple blocking, synchronous receive. */
uint8_t rxByte(void) {
	/* wait for incoming data */
	while(!isRxAvailable());
	/* return register value */
	return UDR0;
}

/* Simple blocking, synchronous transmission. */
void txByte(uint8_t data) {
	/* wait for empty transmit buffer */
	while(!isTxAvailable());
	/* send data */
	UDR0 = data;
}

#else

/* Interrupt handler for non-blocking receipt. */
ISR(USART_RX_vect)
{
    /* move head */
	rxHead = (rxHead + 1) & UART_RX_BUFFER_MASK;

	/* buffer overflow, push tail along */
	if (rxHead == rxTail) {
		rxTail = (rxTail + 1) & UART_RX_BUFFER_MASK;
	}

	/* store received data in buffer */
	rxBuffer[rxHead] = UDR0;
}

/* Interrupt handler for non-blocking transmit. */
ISR(USART_UDRE_vect)
{
	/* check if all data is transmitted */
	if (txHead != txTail) {
		txTail = (txTail + 1) & UART_TX_BUFFER_MASK;
		UDR0 = txBuffer[txTail];
	} else {
		/* disable UDRE interrupt */
		UCSR0B &= ~(1<<UDRIE0);
	}
}

/* Test is there are unread data in the Rx buffer. */
uint8_t isRxAvailable(void) {
	return (rxHead != rxTail);
}

/* Test if Tx is ready to receive new data. */
uint8_t isTxAvailable(void) {
	return (txHead != txTail);
}

/* Non-blocking, interrupt based receipt. */
uint8_t rxByte(void)
{
	uint8_t newTail;

	/* wait for incoming data */
	while (!isRxAvailable());
	/* calculate buffer index */
	newTail = (rxTail + 1) & UART_RX_BUFFER_MASK;
	/* store new index */
	rxTail = newTail;
	/* return data */
	return rxBuffer[newTail];
}

/* Non-blocking, interrupt based transmission. */
void txByte(uint8_t data)
{
	uint8_t newHead;

	/* calculate buffer index */
	newHead = (txHead + 1) & UART_TX_BUFFER_MASK;
	/* wait for free space in buffer */
	while (newHead == txTail);
	/* store data in buffer */
	txBuffer[newHead] = data;
	/* store new index */
	txHead = newHead;
	/* enable UDRE interrupt */
	UCSR0B |= (1<<UDRIE0);
}

#endif

#ifndef UART_H_
#define UART_H_

#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* default Baud rate */
#ifndef BAUD
#define BAUD 19200
#endif

/* Takes the defined BAUD and F_CPU, calculates the prescaler, and configures the hardware USART.
 * The USART gets initialized to asynchronous mode, no parity bit, 1 stop bit, 8 data bits.
 * The function also enables interrupts by calling sei() for non-simple implementation. */
void initUart(void);

/*  Transmit and receive functions. */
void txByte(uint8_t data);
uint8_t rxByte(void);

/* Tx and Rx availability functions. */
uint8_t isRxAvailable(void);
uint8_t isTxAvailable(void);

#ifdef __cplusplus
}
#endif

#endif /* UART_H_ */


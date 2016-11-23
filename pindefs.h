#ifndef PINDEFS_H_
#define PINDEFS_H_

#define SET_INPUT_MODE(ddr,bit) ddr &= ~(1<<bit)
#define SET_OUTPUT_MODE(ddr,bit) ddr |= (1<<bit)
#define SET_HIGH(port,bit) port |= (1<<bit)
#define SET_LOW(port,bit) port &= ~(1<<bit)

/* Pin controlling the servo */
#define SERVO_DDR 	DDRB
#define SERVO_PORT 	PORTB
#define SERVO_PIN 	PINB
#define SERVO_BIT 	PB7

/* IR LED to transcieve the signal */
#define TRC_DDR 	DDRD
#define TRC_PORT 	PORTD
#define TRC_PIN 	PIND
#define TRC_BIT 	PD6

#endif /* PINDEFS_H_ */

#ifndef PINDEFS_H_
#define PINDEFS_H_

#define SET_INPUT_MODE(ddr,bit) ddr &= ~(1<<bit)
#define SET_OUTPUT_MODE(ddr,bit) ddr |= (1<<bit)
#define SET_HIGH(port,bit) port |= (1<<bit)
#define SET_LOW(port,bit) port &= ~(1<<bit)

#define TRC_DDR 	DDRD
#define TRC_PORT 	PORTD
#define TRC_PIN 	PIND
#define TRC_BIT 	PD6

#define SERVO_DDR 	DDRD
#define SERVO_PORT 	PORTD
#define SERVO_PIN 	PIND
#define SERVO_BIT 	PD6

#endif /* PINDEFS_H_ */

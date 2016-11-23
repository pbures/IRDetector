/*
 * config.h
 *
 *  Created on: 6. 11. 2016
 *      Author: pbures
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#define BAUD 19200

#define DEBUG_LEDS
//#define DEBUG_STATE_MACHINE
#define STORE_DATA
#define PRINT_COMMANDS
#define PRINT_HISTOGRAM
//#define USE_TIMER0 //Default is Timer 1.

#define NUM_CHANNELS 6
#define FIRST_CHANNEL 2
#define NUM_READINGS 256

#endif /* CONFIG_H_ */

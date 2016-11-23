/*
 * IRTransciever.h
 *
 *  Created on: 21. 11. 2016
 *      Author: pbures
 */

#ifndef IRTRANSCIEVER_H_
#define IRTRANSCIEVER_H_

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>

#include "pindefs.h"

class IRTransciever {
public:
	IRTransciever();

	static void init();
	static void send(uint8_t byte);

	static void IROn();
	static void IROff();
private:
	static void initialBurst();
	static void inline delay560us();
	static void inline delay1200us();

};

#endif /* IRTRANSCIEVER_H_ */

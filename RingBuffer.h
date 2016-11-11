/*
 * Swapper.h
 *
 *  Created on: 9. 11. 2016
 *      Author: pbures
 */

#ifndef RINGBUFFER_H_
#define RINGBUFFER_H_

template<typename T, uint8_t MAXSIZE>
class RingBuffer {
public:
	RingBuffer() :
			valuesPtr(0), newValues(false) {
	}


	bool add(T *value) {
		values[valuesPtr].copy(value);
		valuesPtr = (valuesPtr + 1) % MAXSIZE;
		newValues = true;
		return true;
	}

	/* Return true if a new value has been added after reset() has been called. */
	bool hasNewCode() {
		return newValues;
	}

	/*
	 * Acknowledges the new values were processed. The hasNewCode() returns false after
	 * this call until a call to add() is made.
	 */
	void reset() {
		newValues = false;
	}

	T values[MAXSIZE];
	uint8_t valuesPtr;

	bool newValues;
};

template<typename T, uint8_t MAXSIZE>
class RingBufferIterator {

public:
	RingBufferIterator(RingBuffer<T, MAXSIZE> *buffer, bool full = false) :
			buffer(buffer) {
		valuesReadPtr = buffer->valuesPtr;
		if (full) {
			valuesReadPtr = (valuesReadPtr + 1) % MAXSIZE;
		}
	}

	T* next() {
		T *ret = 0;
		if (valuesReadPtr != buffer->valuesPtr) {
			ret = &(buffer->values[valuesReadPtr]);
			valuesReadPtr = (valuesReadPtr + 1) % MAXSIZE;
		}
		return ret;
	}

	void print() {
		T *value;
		while (0 != (value = next()))
			value->print();
	}

private:
	RingBuffer<T, MAXSIZE> *buffer;
	uint8_t valuesReadPtr;
};

#endif /* RINGBUFFER_H_ */

#include "main.h"

#define KB_PORT 	GPIOC
#define LED_PORT 	GPIOB

#define NUM_COL 	4
#define NUM_ROW 	4
#define R0MASK		0x1
#define R1MASK		R1MASK << 1
#define R2MASK		R1MASK << 2
#define R3MASK		R1MASK << 3

const char keypad[NUM_ROW * NUM_COL] = {
	'1', '2', '3', 'A',
	'4', '5', '6', 'B',
	'7', '8', '9', 'C',
	'*', '0', '#', 'D'
};

int8_t keypadLoop();

int main() {
	int8_t button;
	uint16_t delay;

	// enable clock for port C and port B
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOCEN | RCC_AHB2ENR_GPIOBEN);

	// drive columns MODE[31:16] / ODR[7:4] and read rows MODE[15:0] / IDR[3:0]
	KB_PORT->MODER &= 0xFFFF0000;
	KB_PORT->MODER |= 0x00005500;

	// set LEDs MODE[7:0] / ODR[3:0] to GPIO output
	LED_PORT->MODER &= 0xFFFFFF00;
	LED_PORT->MODER |= 0x00000055;

	// set pull-down resistors for rows to prevent floating voltages
	KB_PORT->PUPDR &= 0xFFFFFF00;
	KB_PORT->PUPDR |= 0x000000AA;

	// clear LED ODR
	LED_PORT->ODR &= 0xFFFFFFF0;

	button = -1;
	while(1) {
		button = keypadLoop();
		if (button != -1) {
			LED_PORT->ODR &= 0xFFFFFFF0;
			LED_PORT->ODR |= 0x0F & button;
		}
		for (delay = 0; delay < 0xFFFF; delay++);
	}

	return 0;
}

int8_t keypadLoop() {
	uint8_t colShift, row;

	// clear column ODR
	KB_PORT->ODR &= 0xFFFFFF0F;

	// multiplex columns
	for (colShift = 0; 1; colShift = (colShift + 1) % NUM_COL) {
		// drive column
		KB_PORT->BSRR = (0x10 << colShift);

		for (row = 0; row < NUM_ROW; row++) {
			if (KB_PORT->IDR & (1 << row)) {
				return NUM_ROW * row + colShift;
			}
		}

		// disable column
		KB_PORT->BSRR = (0x100000 << colShift);
	}

	return -1;
}

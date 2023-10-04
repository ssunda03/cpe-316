#include "main.h"

int main() {
	uint8_t count;
	uint32_t delay;

	// turn on clock for GPIOC
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

	// set PC[3:0] to output
	GPIOC->MODER &= ~(GPIO_MODER_MODE0);
	GPIOC->MODER |= GPIO_MODER_MODE0_0;

	GPIOC->MODER &= ~(GPIO_MODER_MODE1);
	GPIOC->MODER |= GPIO_MODER_MODE1_0;

	GPIOC->MODER &= ~(GPIO_MODER_MODE2);
	GPIOC->MODER |= GPIO_MODER_MODE2_0;

	GPIOC->MODER &= ~(GPIO_MODER_MODE3);
	GPIOC->MODER |= GPIO_MODER_MODE3_0;

	count = 0;
	while (1) {
		// clear ODR[3:0]
		GPIOC->ODR &= 0xFFF0;
		// set ODR[3:0] to count
		GPIOC->ODR |= (0x000F & count);

		// delay
		for (delay = 0; delay < 100000; delay++);

		// reset count to 0 when > 0xF
		if (++count > 0x000F) {
			count = 0;
		}

	}
}

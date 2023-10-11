#include "main.h"

void TIM2_IRQHandler();
void PartA();
void PartACCR();

volatile uint8_t WAVE_STATE;

int main() {

	PartA();
}

void PartA() {
	// clock and timer configuration
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

	// set LED to waveform
	GPIOA->MODER &= ~GPIO_MODER_MODE5;
	GPIOA->MODER |= GPIO_MODER_MODE5_0;
	GPIOA->ODR &= ~GPIO_PIN_5;

	WAVE_STATE = 0;

	// Output Frequency = Clock Frequency / ((PSC + 1) * (ARR + 1))
	TIM2->PSC = 99;
	TIM2->ARR = 1;
	// enable interrupt
	TIM2->DIER |= TIM_DIER_UIE;

	// enable interrupts
	// ARM Core
	__enable_irq();
	//NVIC enable
	NVIC->ISER[0] = 1 << (TIM2_IRQn & 0x1F); // 28th position on the vector table
	//NVIC_SetPriority(TIM2_IRQn, 0);

	// enable timer in upcounting mode
	TIM2->CR1 &= ~TIM_CR1_CMS & ~TIM_CR1_DIR;
	TIM2->CR1 |= TIM_CR1_CEN;

	while (1) {
		if (WAVE_STATE == 1) {
			GPIOA->BSRR = GPIO_BSRR_BR5;
			WAVE_STATE = 0;
		}
		else if (WAVE_STATE == 2) {
			GPIOA->BSRR = GPIO_BSRR_BS5;
			WAVE_STATE = 0;
		}
	}
}

void PartACCR() {
	// clock and timer configuration
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

	// set LED to waveform
	GPIOA->MODER &= ~GPIO_MODER_MODE5;
	GPIOA->MODER |= GPIO_MODER_MODE5_0;
	GPIOA->ODR &= ~GPIO_PIN_5;

	WAVE_STATE = 0;

	// Output Frequency = Clock Frequency / ((PSC + 1) * (ARR + 1))
	TIM2->PSC = 99;
	TIM2->ARR = 0xFFFFFFFF;
	// enable interrupt
	TIM2->DIER |= TIM_DIER_CCR1IE;

	// enable interrupts
	// ARM Core
	__enable_irq();
	//NVIC enable
	NVIC->ISER[0] = 1 << (TIM2_IRQn & 0x1F); // 28th position on the vector table
	//NVIC_SetPriority(TIM2_IRQn, 0);

	// enable timer in upcounting mode
	TIM2->CR1 &= ~TIM_CR1_CMS & ~TIM_CR1_DIR;
	TIM2->CR1 |= TIM_CR1_CEN;

	while (1) {
		if (WAVE_STATE == 1) {
			GPIOA->BSRR = GPIO_BSRR_BR5;
			WAVE_STATE = 0;
		}
		else if (WAVE_STATE == 2) {
			GPIOA->BSRR = GPIO_BSRR_BS5;
			WAVE_STATE = 0;
		}
	}
}

void TIM2_IRQHandler() {
	static uint8_t cycle = 0;

	if (!cycle) {
		WAVE_STATE = 1;
	}

	else if (cycle == 3) {
		WAVE_STATE = 2;
	}

	TIM2->SR &= ~TIM_SR_UIF;
	cycle = ++cycle % 4;
}

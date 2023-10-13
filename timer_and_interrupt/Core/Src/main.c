#include "main.h"


void TIM2_IRQHandler(void);
void PartA();
void PartB();


int main(void)
{
	//HAL_Init();

	PartB();

}// end main


void PartA() // works well
{
	// turn on clocks
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN; // GPIOC clock
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN; // TIM2 clock

	// set GPIO PC0 as an output
	GPIOC->MODER &= ~GPIO_MODER_MODE0; // clears mode bits on PC0
	GPIOC->MODER |= (1 << GPIO_MODER_MODE0_Pos); // sets PC0 to output
	GPIOC->ODR &= ~GPIO_PIN_0; // sets the output on PC0 to 0

	// configure timer count settings
	TIM2->CR1 &= ~TIM_CR1_CMS; // sets count to be one directional
	TIM2->CR1 &= ~TIM_CR1_DIR; // sets timer to count up
	TIM2->PSC = 0; // divides timer clock by PSC + 1
	TIM2->ARR = 804; // timer counts to ARR + 1 // 399 for 50% (w/o CCR1) // 799 when w CCR1
	TIM2->CCR1 = 594; // compares count to CCR1 // 599 for 25% duty b/c 599 ~= 0.75*799

	// enable interrupts
	__enable_irq(); // enables ARM interrupts
	NVIC->ISER[0] = (1 << (TIM2_IRQn & 0x1F)); // enables NVIC TIM2 interrupt
	TIM2->SR &= ~TIM_SR_UIF; // resets TIM2 update interrupt flag
	TIM2->SR &= ~TIM_SR_CC1IF; // resets TIM2 CC1 interrupt flag
	TIM2->DIER |= TIM_DIER_UIE; // enable TIM2 update interrupt
	TIM2->DIER |= TIM_DIER_CC1IE; // enable TIM2 CC1 interrupt

	// start timer
	TIM2->CR1 |= TIM_CR1_CEN; // enables the counter

	// wait for interrupts
	while(1);

}


void TIM2_IRQHandler(void)
{
	// reset interrupt flag
	if(TIM2->SR & TIM_SR_UIF)
	{
		TIM2->SR &= ~TIM_SR_UIF;
	}
	else if(TIM2->SR & TIM_SR_CC1IF)
	{
		TIM2->SR &= ~TIM_SR_CC1IF;
	}

	// toggle output
	GPIOC->ODR ^= GPIO_PIN_0;

	// for part B only
	TIM2->CNT = 0x0; // resets the count
}


void PartB() // ARR value so big that takes forever to restart - how do we reset count? (set the CNT reg)
{
	// turn on clocks
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN; // GPIOC clock
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN; // TIM2 clock

	// set GPIO PC0 as an output
	GPIOC->MODER &= ~GPIO_MODER_MODE0; // clears mode bits on PC0
	GPIOC->MODER |= (1 << GPIO_MODER_MODE0_Pos); // sets PC0 to output
	GPIOC->ODR &= ~GPIO_PIN_0; // sets the output on PC0 to 0

	// set GPIO PC1 as an output
	GPIOC->MODER &= ~GPIO_MODER_MODE1; // clears mode bits on PC1
	GPIOC->MODER |= (1 << GPIO_MODER_MODE1_Pos); // sets PC1 to output
	GPIOC->ODR &= ~GPIO_PIN_1; // sets the output on PC1 to 0

	// configure timer count settings
	TIM2->CR1 &= ~TIM_CR1_CMS; // sets count to be one directional
	TIM2->CR1 &= ~TIM_CR1_DIR; // sets timer to count up
	TIM2->PSC = 0; // divides timer clock by PSC + 1
	TIM2->ARR = 0xFFFFFFFF; // ARR set so will continuously count
	TIM2->CCR1 = 344; // CCR1 to create a 50% duty cycle

	// enable interrupts
	__enable_irq(); // enables ARM interrupts
	NVIC->ISER[0] = (1 << (TIM2_IRQn & 0x1F)); // enables NVIC TIM2 interrupt
	TIM2->SR &= ~TIM_SR_UIF; // resets TIM2 update interrupt flag
	TIM2->SR &= ~TIM_SR_CC1IF; // resets TIM2 CC1 interrupt flag
	TIM2->DIER |= TIM_DIER_UIE; // enable TIM2 update interrupt
	TIM2->DIER |= TIM_DIER_CC1IE; // enable TIM2 CC1 interrupt

	// start timer
	TIM2->CR1 |= TIM_CR1_CEN; // enables the counter

	// wait for interrupts
	while(1);
}






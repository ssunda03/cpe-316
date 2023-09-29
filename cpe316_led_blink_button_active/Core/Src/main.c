#include "main.h"

int main(void)
{

	// turns on clock to GPIO banks A and C
  RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOCEN);

  // bank A as GPIO mode
  GPIOA->MODER &= ~(GPIO_MODER_MODE5);
  GPIOA->MODER |= (GPIO_MODER_MODE5_0);

  // bank C as GPIO mode
  GPIOC->MODER &= ~(GPIO_MODER_MODE13);
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD13);

  while (1) {
	  // if BUTTON not pressed (high), clear LED, else set LED
	  GPIOA->ODR = GPIOC->IDR & GPIO_PIN_13 ? GPIOA->ODR & ~GPIO_PIN_5 : GPIOA->ODR | GPIO_PIN_5;

  } // end while
}// end main

#include "main.h"

int main(void)
{
	HAL_Init();

	// turns on clock to GPIO banks A and C
  RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOCEN);

  // bank A as GPIO mode
  GPIOA->MODER &= ~(GPIO_MODER_MODE5);
  GPIOA->MODER |= (GPIO_MODER_MODE5_0);

  // bank C as GPIO mode
  GPIOC->MODER &= ~(GPIO_MODER_MODE13);
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD13);

  while (1)
  {
		  GPIOA->ODR &= ~GPIO_PIN_5;	// set pin 5 of PORT A to 0
		  HAL_Delay(100);
	      GPIOA->ODR |= GPIO_PIN_5;			// set pin 5 of PORT A to 1
	      HAL_Delay(100);

  } // end while
}// end main

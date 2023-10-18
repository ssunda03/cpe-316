#include "main.h"
#define VOUT_TO_D(x) 0x3000 | (0xFFF & (int)((double)x / 3.3 * 0xFFF))

void DAC_Init();
void DAC_Write(uint16_t D);
void DAC_Convert();

int main(void)
{
  HAL_Init();
  DAC_Init();
  uint16_t spi_data_low = DAC_Convert(1);
  uint16_t spi_data_high = DAC_Convert(2);

  uint32_t period;
  period = 4;
  while (1)
  {
	  for (int i = 0; i < 3 * period / 4; i++) {
		  DAC_Write(spi_data_low);
	  }

	  for (int i = 0; i < period / 4; i++) {
		  DAC_Write(spi_data_high);
  	  }
  }
}

void DAC_Init() {
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL4 | GPIO_AFRL_AFSEL5 | GPIO_AFRL_AFSEL7);
	GPIOA->AFR[0] |= ((5 << GPIO_AFRL_AFSEL4_Pos) | (5 << GPIO_AFRL_AFSEL5_Pos) | (5 << GPIO_AFRL_AFSEL7_Pos));

	GPIOA->MODER &= ~(GPIO_MODER_MODE4 | GPIO_MODER_MODE5 | GPIO_MODER_MODE7);
	GPIOA->MODER |= GPIO_MODER_MODE4_1 | GPIO_MODER_MODE5_1 | GPIO_MODER_MODE7_1;


	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	SPI1->CR1 |= SPI_CR1_MSTR;
	SPI1->CR2 = SPI_CR2_SSOE | SPI_CR2_NSSP | (0xF << SPI_CR2_DS_Pos);
	SPI1->CR1 |= SPI_CR1_SPE;
}

void DAC_Write(uint16_t D) {
	while(!(SPI1->SR & SPI_SR_TXE));
	SPI1->DR = D;
}

uint16_t DAC_Convert(int32_t Vout) {
	return (
		0x3000 | (0xFFF & (uint16_t)(Vout * 1000 / 3300 * 0xFFF))
	);

}

/*#include "main.h"

void DAC_Init();
void DAC_Write();
void DAC_Convert();

int main() {
	HAL_Init();

	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

	// SPI SCK / MOSI
	GPIOA->MODER &= ~(GPIO_PIN_5 | GPIO_PIN_7);
	// GPIO Alternate Function (SPI)
	GPIOA->MODER |= (0b10 << (2 * 5)) | (0b11 << (2 * 7));
	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL5 | GPIO_AFRL_AFSEL7);
	GPIOA->AFR[0] |= (0b0101 << (4 * 5)) | (0b0101 << (4 * 7));

	// CPOL = CPHA = 0
	SPI1->CR1 &= ~0b11;
	SPI1->CR1 |= SPI_CR1_MSTR;

	while(1) {

	}
	return 0;
}*/

#include "main.h"
#define VOUT_TO_D(x) 0x3000 | (0xFFF & (int)((double)x / 3.3 * 0xFFF))

int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /*
   * Configure SPI Pins		PA4 - SPI_1_NSS		PA5 - SPI_1_SCK
   * 						PA6 - SPI_1_MISO	PA7 - SPI_1_MOSI
   * follow order of configuring registers AFR, OTYPER, PUPDR, OSPEEDR, MODDER
   * to avoid a glitch is created on the output pin
   */
  RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN);
  GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL4 |
		  	  	  	 GPIO_AFRL_AFSEL5 |		// mask AF selection
		  	  	  	 GPIO_AFRL_AFSEL7);
  GPIOA->AFR[0] |= ((5 << GPIO_AFRL_AFSEL4_Pos) |				// select SPI_1 (AF5)
  		  	  	    (5 << GPIO_AFRL_AFSEL5_Pos) |
				    (5 << GPIO_AFRL_AFSEL7_Pos));

  GPIOA->MODER &= ~(GPIO_MODER_MODE4 |
		  	  	  	GPIO_MODER_MODE5 |		// mask function
		  	  	  	GPIO_MODER_MODE7);
  GPIOA->MODER |= (GPIO_MODER_MODE4_1 |
		  	  	   GPIO_MODER_MODE5_1 |		// enable alternate function
		  	  	   GPIO_MODER_MODE7_1);

  // configure SPI 1
  RCC->APB2ENR |= (RCC_APB2ENR_SPI1EN);		// enable SPI1 clock
  SPI1->CR1 |= SPI_CR1_MSTR;				// enable master mode, fck/2, hardware CS, MSB first, full duplex
  SPI1->CR2 = (SPI_CR2_SSOE |				// enable CS output
		  	   SPI_CR2_NSSP |				// create CS pulse
			   (0xF << SPI_CR2_DS_Pos));	// 16-bit data frames
  SPI1->CR1 |= (SPI_CR1_SPE);				// enable SPI


  uint16_t spi_data_low = VOUT_TO_D(1);
  uint16_t spi_data_high = VOUT_TO_D(2);
  uint16_t temp;

  while (1)
  {
	  for (int i = 0; i < 3; i++) {
		  while(!(SPI1->SR & SPI_SR_TXE));		// ensure room in TXFIFO before writing
		  SPI1->DR = spi_data_low;
	  }					// clear RX FIFO

	  while(!(SPI1->SR & SPI_SR_TXE));		// ensure room in TXFIFO before writing
	  SPI1->DR = spi_data_high;					// clear RX FIFO

  }
}

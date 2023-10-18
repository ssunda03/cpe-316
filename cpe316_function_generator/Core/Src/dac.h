/*
 * dac.h
 *
 *  Created on: Oct 17, 2023
 *      Author: jackkrammer
 *
 *  header file for the SPI1 connection to the MCP4921 DAC
 */

#ifndef SRC_DAC_H_
#define SRC_DAC_H_


void DAC_init(); // initializes the SPI for the DAC
void DAC_write(uint16_t data); // writes a 16 bit value to the DAC
uint16_t DAC_volt_conv(int mv);/* converts an int representing millivolts to a 16 bit int DAC value,
   	   	   	   	   	   	   	   	  maxes out at the highest voltage possible on the DAC */


/* converts an int representing millivolts to a 16 bit int DAC value
   maxes out at the highest voltage possible on the DAC */
uint16_t DAC_volt_conv(int mv)
{
	return ( 0x3000 | (0xFFF & (uint16_t)(mv / 3300 * 0xFFF) ) );
	// control bits | bit mask of 16 bit int version of the ratio of input to max millivolt
}

// writes a 16 bit value to the DAC
void DAC_write(uint16_t mv)
{
	while(!(SPI1->SR & SPI_SR_TXE));		// ensure room in TXFIFO before writing
	SPI1->DR = mv;
}

// initializes the SPI for the DAC
void DAC_init()
{
	/*
	* Configure SPI Pins		PA4 - SPI_1_NSS		PA5 - SPI_1_SCK
	* 							PA6 - SPI_1_MISO	PA7 - SPI_1_MOSI
	* follow order of configuring registers AFR, OTYPER, PUPDR, OSPEEDR, MODDER
	* to avoid a glitch is created on the output pin
	*/
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN);
	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL4 | GPIO_AFRL_AFSEL5 |	// mask AF selection
					  GPIO_AFRL_AFSEL6 | GPIO_AFRL_AFSEL7);
	GPIOA->AFR[0] |= ((5 << GPIO_AFRL_AFSEL4_Pos) |				// select SPI_1 (AF5)
					(5 << GPIO_AFRL_AFSEL5_Pos) |
					(5 << GPIO_AFRL_AFSEL6_Pos) |
					(5 << GPIO_AFRL_AFSEL7_Pos));
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT4 | GPIO_OTYPER_OT5 |		// push-pull output
					  GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7);
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD4 | GPIO_PUPDR_PUPD5 |		// no pull ups or pull downs
					GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD7);
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED4 | 					// low speed
					  GPIO_OSPEEDR_OSPEED5 |
					  GPIO_OSPEEDR_OSPEED6 |
					  GPIO_OSPEEDR_OSPEED7);
	GPIOA->MODER &= ~(GPIO_MODER_MODE4 | GPIO_MODER_MODE5 |		// mask function
					GPIO_MODER_MODE6 | GPIO_MODER_MODE7);
	GPIOA->MODER |= (GPIO_MODER_MODE4_1 | GPIO_MODER_MODE5_1 |	// enable alternate function
				   GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1);

	// configure SPI 1
	RCC->APB2ENR |= (RCC_APB2ENR_SPI1EN);	// enable SPI1 clock
	SPI1->CR1 = (SPI_CR1_MSTR);				// enable master mode, fck/2, hardware CS, MSB first, full duplex
	SPI1->CR2 = (SPI_CR2_SSOE |				// enable CS output
			   SPI_CR2_NSSP |				// create CS pulse
			   (0xF << SPI_CR2_DS_Pos));	// 16-bit data frames
	SPI1->CR1 |= (SPI_CR1_SPE);				// enable SPI
}

#endif /* SRC_DAC_H_ */

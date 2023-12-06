/*
 * Jack Krammer
 *
 * tests the TSC2046 resistive touch screen chip
 *
 */


// includes
#include "main.h"
#include <stdio.h>
#include "uart.h"

// defines
#define Y_ADDR_BITS  0b001
#define Z1_ADDR_BITS 0b011
#define Z2_ADDR_BITS 0b100
#define X_ADDR_BITS  0b101

#define X_RESISTANCE 400 // *********** MEASURE THIS **************************************************************
#define THRESHOLD 1000
#define HIGH 1
#define LOW 0
#define BUFF_SIZE 16

// typedefs
typedef struct TSpoint{
	uint16_t x, y;
	float z;
} TSpoint;

// function prototypes
void TSC_init(); // initializes the pins for TSC2046 device
uint16_t read_coordinate(uint8_t coord); // makes the control byte from the input coordinate and returns the TSC output
void write_byte(uint8_t data); // writes a byte to SPI
uint8_t read_byte(uint8_t dummy_val); // reads a byte from SPI using the dummy value to initiate
uint16_t TSC_conversion(uint8_t control_byte); // based on the control byte, returns the TSC output value
TSpoint get_point(); // returns a point struct of the measured values from TSC2046
uint8_t is_touched(); // returns 1 if is touched and 0 if not
void TSC_EXTI_interrupt_init(); // initializes the EXTI interrupt for the /PENIRQ pin to be connected to PA0
void EXTI0_IRQHandler(); // EXTI0 interrupt handler
void disable_NVIC_interrupt(uint32_t interrupt_num); // disables the NVIC interrupt for the input interrupt number
void enable_NVIC_interrupt(uint32_t interrupt_num); // enables the NVIC interrupt for the input interrupt number
//void PA8_LED_init(); // initializes a GPIO output on pin PA8
//void set_PA8_LED(uint8_t val); // sets the LED on if val > 0 and off if val == 0
void PA9_LED_init(); // initializes a GPIO output on pin PA9
void set_PA9_LED(uint8_t val); // sets the LED on if val > 0 and off if val == 0
void int_to_str(int num, char* buff); // converts and int and returns a string of length 8
void USART_print_int(int num); // prints an integer over USART
void set_TSC_CS(uint8_t val); // sets the CS of the TSC to the desired value
uint16_t TSC_conversion2(uint8_t control_byte); // second version of the TSC conversion function, writes control byte and returns output


// global variables
volatile uint8_t screen_touched = 0;



int main()
{
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	// initialize the TSC2046 pins
	TSC_init();

	// initialize the UART connection with a clear screen and cursor top left
	USART_init();

	// prints "hello" to the USART interface
	USART_Print("hello there\n\r");
//	USART_Print("there\n\r");


	// initialize the LEDs
	PA8_LED_init(); // this is the RED LED
	PA9_LED_init(); // this is the YELLOW LED

	// variables
	TSpoint pt = {.x = 0, .y = 0, .z = 0};
	char buff[BUFF_SIZE];


	while(1)
	{
		// prints the x coordinate of the touch
		int_to_str(pt.x, buff);
		USART_Print("  x: ");
		USART_Print(buff);

		// prints the y coordinate of the touch
		int_to_str(pt.y, buff);
		USART_Print("  y: ");
		USART_Print(buff);

		// prints the z (pressure) coordinate of the touch
		int_to_str(pt.z, buff);
		USART_Print("  z: ");
		USART_Print(buff);

		// prints the global touch flag set by the /PENIRQ interrupt
		int_to_str(screen_touched, buff);
		USART_Print("  touch: ");
		USART_Print(buff);
		USART_Print("\n\r");

		HAL_Delay(1000);

		if(screen_touched) // putting this here resulted in touch always = 1, meaning the get_point function is causing the interrupt somehow
		{
			screen_touched = 0;
		}

		// gets the current point from the screen
		pt = get_point();

	}


	return 0;
}

// prints an integer over USART
void USART_print_int(int num)
{
	char buff[BUFF_SIZE];
	int_to_str(num, buff);
	USART_Print(buff);
}

// converts and int and returns a string of length 8
void int_to_str(int num, char* buff)
{
	snprintf(buff, /*sizeof(buff)*/BUFF_SIZE, "%d", num);
}

// sets the LED on if val > 0 and off if val == 0
void set_PA9_LED(uint8_t val)
{
	if(val > 0)
		GPIOA->ODR |= (1 << 9);
	else
		GPIOA->ODR &= ~(1 << 9);
}

// sets the LED on if val > 0 and off if val == 0
//void set_PA8_LED(uint8_t val)
//{
//	if(val > 0)
//		GPIOA->ODR |= (1 << 8);
//	else
//		GPIOA->ODR &= ~(1 << 8);
//}

// initializes a GPIO output on pin PA9
void PA9_LED_init()
{
	// enable GPIOA clock
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN);

	// set GPIO mode
	GPIOA->MODER &= ~(GPIO_MODER_MODE9);
	GPIOA->MODER |= (GPIO_MODER_MODE9_0);

	// initialize output to be off
	GPIOA->ODR &= ~(1 << 9);
}

//// initializes a GPIO output on pin PA8
//void PA8_LED_init()
//{
//	// enable GPIOA clock
//	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN);
//
//	// set GPIO mode
//	GPIOA->MODER &= ~(GPIO_MODER_MODE8);
//	GPIOA->MODER |= (GPIO_MODER_MODE8_0);
//
//	// initialize output to be off
//	GPIOA->ODR &= ~(1 << 8);
//}

// returns 1 if is touched and 0 if not
uint8_t is_touched()
{
	TSpoint pt = get_point();

	return pt.z < THRESHOLD && pt.z != 0;
}

// returns a point struct of the measured values from TSC2046
TSpoint get_point()
{
	// DISABLE INTERRUPTS
	disable_NVIC_interrupt(EXTI0_IRQn);
	__disable_irq();

	uint16_t x = read_coordinate(X_ADDR_BITS);
	uint16_t y = read_coordinate(Y_ADDR_BITS);
	uint16_t z1 = read_coordinate(Z1_ADDR_BITS);
	uint16_t z2 = read_coordinate(Z2_ADDR_BITS);

	// ENABLE INTERRUPTS
	enable_NVIC_interrupt(EXTI0_IRQn);
	__enable_irq();

	float z_pressure
		= X_RESISTANCE * ((float)x / 4096.f) * ((float)z2 / (float)z1 - 1.f);

	TSpoint pt;
	pt.x = x;
	pt.y = y;
	pt.z = z_pressure;

	return pt;
}

// makes the control byte from the input coordinate and returns the TSC output
uint16_t read_coordinate(uint8_t coord)
{
	// variables
	uint8_t ctrl = 0;
	uint8_t addr_bits = coord << 4;
	// set start bit high
	ctrl |= 0x80;

	// add address bits to control byte
	ctrl |= addr_bits;

	// set mode bit low for 12-bit ADC resolution
	ctrl &= ~(1 << 3);
	// set SER/DFR bit low for differential reference for ADC
	ctrl &= ~(1 << 2);
//	// set SER/DFR bit high for single-ended mode for ADC
//	ctrl |= (1 << 2);
	// sets power down bits low to enable interrupt and conversions with internal reference
	ctrl &= ~(3);

	// returns the device output from this control byte
	return TSC_conversion2(ctrl);
}

// second version of the TSC conversion function, writes control byte and returns output
uint16_t TSC_conversion2(uint8_t control_byte)
{
	// variables
	uint8_t data[2];
	uint16_t combined;

	// set chip select pin low
	set_TSC_CS(LOW);

	// sends control byte
	write_byte(control_byte);

	// read data from device
	data[0] = read_byte(0); // reads the MSB
	data[1] = read_byte(0); // reads the LSB

	// set chip select pin high
	set_TSC_CS(HIGH);

	// combine the data read
	combined |= (((uint16_t)data[0]) << 8);	// adds the MSB
	combined |= ((uint16_t)data[1]);		// adds the LSB

	// adjusts combined for alignment
	combined = ((combined >> 3) & 0x0FFF);

	// returns the combined reads
	return combined;
}

// sets the CS of the TSC to the desired value
void set_TSC_CS(uint8_t val)
{
	if(val) GPIO->ODR |=  (1 << 4);
	else 	GPIO->ODR &= ~(1 << 4);
}

// based on the control byte, returns the TSC output value
uint16_t TSC_conversion(uint8_t control_byte)
{
	// variables
	uint8_t data[2];
	uint16_t combined = 0;

	// sends control byte
	write_byte(control_byte);

	// read data from device
	data[0] = read_byte(0); // reads the MSB
	data[1] = read_byte(0); // reads the LSB

	USART_print_int(data[0]);
	USART_Print("-");

	USART_print_int(data[1]);
	USART_Print("-");

	// combine the data read
	combined |= (((uint16_t)data[0]) << 8);	// adds the MSB
	combined |= ((uint16_t)data[1]);		// adds the LSB

	USART_print_int(combined);
	USART_Print("-");

	// adjusts combined for alignment
	combined = ((combined >> 3) & 0x0FFF);

	USART_print_int(combined);
	USART_Print(" ");

	// returns the combined reads
	return combined;
}

// reads a byte from SPI using the dummy value to initiate
uint8_t read_byte(uint8_t dummy_val)
{
	// variables
	uint8_t data;

	// dummy write then read
	while(!(SPI1->SR & SPI_SR_TXE));		// ensure room in TXFIFO before writing
	SPI1->DR = dummy_val;
	while(!(SPI1->SR & SPI_SR_RXNE)); 		// wait to receive 8-bits
	data = SPI1->DR;						// clear RX FIFO

	// returns byte read
	return data;
}

// writes a byte to SPI
void write_byte(uint8_t data)
{
	while(!(SPI1->SR & SPI_SR_TXE));		// ensure room in TXFIFO before writing
	SPI1->DR = data;
}

// EXTI0 interrupt handler
void EXTI0_IRQHandler(void)
{
    // Check if the interrupt was triggered by EXTI0
    if (EXTI->PR1 & EXTI_PR1_PIF0)
    {
    	// set the flag for the while loop to get a conversion
        screen_touched = 1;

        // clear the interrupt flag
        EXTI->PR1 |= EXTI_PR1_PIF0;
    }
}

// initializes the EXTI interrupt for the /PENIRQ pin to be connected to PA0
void TSC_EXTI_interrupt_init()
{
	/*
	 * sets GPIO pin PA0 as an EXTI input interrupt to interface the TSC2046 /PENIRQ pin
	 *
	 * /PENIRQ has a pull up resistor, and indicates the interrupt when the pin goes low
	 * 		configure PA0 as an input with pull up resistor
	 */

	// enable the GPIOA peripheral clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

	// configure PA0 as an input with pull-up
	GPIOA->MODER &= ~GPIO_MODER_MODE0; 	// sets to input
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD0;	// sets to no pull-up and no pull-down
//	GPIOA->PUPDR |= GPIO_PUPDR_PUPD0_0; // sets to have a pull-up

	// enable the SYSCFG peripheral clock
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	// Connect EXTI0 line to PA0
	SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0_Msk;

	// configure EXTI0 to trigger on falling edge
	EXTI->FTSR1 |= EXTI_FTSR1_FT0;

	// enable EXTI0 interrupt
	EXTI->IMR1 |= EXTI_IMR1_IM0;

	// enable EXTI0 interrupt in NVIC
//	NVIC_EnableIRQ(EXTI0_IRQn); // does same thing as below
//	NVIC->ISER[0] |= (1 << (EXTI0_IRQn & 0x7)); // does the same thing as below
	enable_NVIC_interrupt(EXTI0_IRQn);


	// enable arm interrupts
	__enable_irq();
}

// enables the NVIC interrupt for the input interrupt number
void enable_NVIC_interrupt(uint32_t interrupt_num)
{
	NVIC->ISER[interrupt_num / 32] |= (1 << (interrupt_num % 32));
}

// disables the NVIC interrupt for the input interrupt number
void disable_NVIC_interrupt(uint32_t interrupt_num)
{
	NVIC->ICER[interrupt_num / 32] |= (1 << (interrupt_num % 32));
}

// initializes the pins for TSC2046 device
void TSC_init()
{
	/*
	* Configure TSC Pins		PA4 - SPI_1_NSS		PA5 - SPI_1_SCK		PA8 - TSC_BUSY
	* 							PA6 - SPI_1_MISO	PA7 - SPI_1_MOSI
	* follow order of configuring registers AFR, OTYPER, PUPDR, OSPEEDR, MODDER
	* to avoid a glitch is created on the output pin
	*/
	// turn on GPIOA peripheral clock
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN);
	// mask AF selection
	GPIOA->AFR[0] &= ~( //GPIO_AFRL_AFSEL4 |
						GPIO_AFRL_AFSEL5 |
						GPIO_AFRL_AFSEL6 |
						GPIO_AFRL_AFSEL7 );
	// select SPI_1 (AF5)
	GPIOA->AFR[0] |= ( 	//(5 << GPIO_AFRL_AFSEL4_Pos) |
						(5 << GPIO_AFRL_AFSEL5_Pos) |
						(5 << GPIO_AFRL_AFSEL6_Pos) |
						(5 << GPIO_AFRL_AFSEL7_Pos) );
	// push-pull output
	GPIOA->OTYPER &= ~(	GPIO_OTYPER_OT4 |
						GPIO_OTYPER_OT5 |
						GPIO_OTYPER_OT6 |
						GPIO_OTYPER_OT7 );
	// no pull ups or pull downs
	GPIOA->PUPDR &= ~(	GPIO_PUPDR_PUPD4 |
						GPIO_PUPDR_PUPD5 |
						GPIO_PUPDR_PUPD6 |
						GPIO_PUPDR_PUPD7 );
	// low speed
	GPIOA->OSPEEDR &= ~( GPIO_OSPEEDR_OSPEED4 |
					 	 GPIO_OSPEEDR_OSPEED5 |
						 GPIO_OSPEEDR_OSPEED6 |
						 GPIO_OSPEEDR_OSPEED7 );
	// mask function
	GPIOA->MODER &= ~(	GPIO_MODER_MODE4 |
						GPIO_MODER_MODE5 |
						GPIO_MODER_MODE6 |
						GPIO_MODER_MODE7 |
						GPIO_MODER_MODE8 ); // this line is for PA8, TSC BUSY pin, configures to be input
	// enable alternate function
	GPIOA->MODER |= (	//GPIO_MODER_MODE4_1 |
						GPIO_MODER_MODE5_1 |
						GPIO_MODER_MODE6_1 |
						GPIO_MODER_MODE7_1 );
	// make pin PA4 a regular GPIO output
	GPIO->MODER |= GPIO_MODER_MODE4_0;
	// initialize output on PA4 to be HIGH
	GPIO->ODR |= (1 << 4);

	// configure SPI 1
	RCC->APB2ENR |= (RCC_APB2ENR_SPI1EN);	// enable SPI1 clock
	SPI1->CR1 = (SPI_CR1_MSTR);				// enable master mode, fck/2, hardware CS, MSB first, full duplex
	SPI1->CR1 &= ~(SPI_CR1_CPHA | 			// CPHA = 0
				SPI_CR1_CPOL); 				// CPOL = 0
	SPI1->CR2 = (SPI_CR2_SSOE |				// enable CS output
			   SPI_CR2_NSSP |				// create CS pulse
			   (0x7 << SPI_CR2_DS_Pos));	// 8-bit data frames
	SPI1->CR1 |= (SPI_CR1_SPE);				// enable SPI

	// enables the EXTI GPIO pin interrupt on PA0
	TSC_EXTI_interrupt_init();
}



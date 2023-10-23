/*
 * Jack Krammer and Srini Sundararaman
 * October 22, 2023
 * CPE 316 Project 1
 * Function Generator
 */

#include "main.h"
#include "keypad_12.h"
#include "dac.h"
#include "timer2.h"
#include "sine_table_DAC_600_2.h"
#include "ramp_table_DAC_600_2.h"
#include "triangle_table_DAC_600_2.h"

// enums
typedef enum WAVE_TYPE {
		SQUARE 	 	= 0,
		SINE 	 	= 1,
		RAMP 		= 2,
		TRIANGLE 	= 3
} WAVE_TYPE;

// defines
#define SYSCLK_FREQ 32 // frequency of the system clock in MHz
#define TABLE_SIZE 600 // the max number of points we want to use for 100Hz waves, (L)CM of 3,4,5
#define STRIDE_SCALE 1 // number to scale the strides of the lookup tables

// global variables
volatile uint8_t ISR_flag = 0; // flag that tells main that ISR just happened

// function prototypes
void sysclk_init(); // configures the system clock to be 32MHz
void test_DAC(); // tests the DAC
void TIM2_IRQHandler(void); // interrupt handler for TIM2



int main()
{
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	// configures the system clock to be 32MHz
	sysclk_init();

	// initializes keypad
	keypad_init();

	// initializes SPI and DAC
	DAC_init();
	//test_DAC();

	// initializes the TIM2 interrupt and starts the count
	TIM2_init(0x215, 0, 0xFFFFFFFF); // arr, psc, ccr1(configured off in header file)

//	GPIOC->MODER &= ~GPIO_MODER_MODE8; // clears PC8 pin
//	GPIOC->MODER |= (1 << GPIO_MODER_MODE8_Pos); // sets PC8 to output
//	GPIOC->ODR &= ~GPIO_ODR_OD8_Pos;


	// wave variables
	WAVE_TYPE wave = SQUARE;
	uint8_t duty = 50;
	uint8_t sq_status = 0;
	uint16_t sq_low = DAC_volt_conv(100); // 0V
	uint16_t sq_high = DAC_volt_conv(2930); // 3V

	// keypad variables
	int8_t kp_ret = -1;
	uint8_t kp_val = 0xF;

	// lookup table variables
	uint16_t stride_length = 1; // amount to increment lookup table index
	uint16_t table_index = 0; // index of the look up table --> need as global?
	uint16_t val = 0;

	while(1)
	{
		// every interval write to DAC
		if(ISR_flag)
		{
			switch(wave)
			{
			case SQUARE:
				sq_status = table_index * 100 / TABLE_SIZE;
				if(sq_status < duty)
				{
					val = sq_high;
				}
				else
				{
					val = sq_low;
				}
//				if(sq_status) // output is currently high
//				{
//					val = sq_low; // set the output low
//					sq_status = 0;
//				}
//				else // output is currently low
//				{
//					val = sq_high; // set the output high
//					sq_status = 1;
//				}
				break;

			case SINE:
				val = sine_table[table_index];
				break;

			case RAMP:
				val = ramp_table[table_index];
				break;

			case TRIANGLE:
				val = triangle_table[table_index];
				break;

			default:
				// do 50% duty square wave
				break;
			}
			// write to DAC
			DAC_write(val);
			// increment index
			table_index = (table_index + (stride_length * STRIDE_SCALE)) % TABLE_SIZE;
			// reset ISR_flag
			ISR_flag = 0;
		}

		// loop through keyboard once
		kp_ret = loop_keypad_once(); // returns index of button pressed or -1 if no button press detected
		if(kp_ret != -1) // button was pressed
		{
			kp_val = keypad_vals[kp_ret];
			switch(kp_val)
			{
			case 0x1:
				// set freq to 100Hz
				stride_length = 1;
				break;
			case 0x2:
				// set freq to 200Hz
				stride_length = 2;
				break;
			case 0x3:
				// set freq to 300Hz
				stride_length = 3;
				break;
			case 0x4:
				// set freq to 400Hz
				stride_length = 4;
				break;
			case 0x5:
				// set freq to 500Hz
				stride_length = 5;
				break;
			case 0x6:
				// change output to sine waveform
				wave = SINE;
				stride_length = 1;
				table_index = 0;
				break;
			case 0x7:
				// change output to triangle waveform
				wave = TRIANGLE;
				stride_length = 1;
				break;
			case 0x8:
				// change output to sawtooth (ramp) waveform
				wave = RAMP;
				stride_length = 1;
				table_index = 0;
				break;
			case 0x9:
				// change output to square waveform
				wave = SQUARE;
				duty = 50;
				stride_length = 1;
				table_index = 0;
				break;
			case 0x0:
				// if square wave, reset duty cycle to 50%
				if(wave == SQUARE)
				{
					duty = 50;
				}
				break;
			case 0xA: // the '*' button
				// if square wave, decrease duty cycle by 10%, down to a minimum of 10%
				if(wave == SQUARE && duty > 10)
				{
					duty -= 10;
				}
				HAL_Delay(200);
				break;
			case 0xB:
				// if square wave, increase duty cycle by 10%, up to a maximum of 90%
				if(wave == SQUARE && duty < 90)
				{
					duty += 10;
				}
				HAL_Delay(200);
				break;
			default:
				// not an expected value
				break;
			}
		}

	} // end while

	return 0;
}



// interrupt handler for TIM2
void TIM2_IRQHandler(void)
{
//	GPIOC->ODR ^= (1 << GPIO_ODR_OD8_Pos);

	// reset interrupt flag
	if(TIM2->SR & TIM_SR_UIF) // from ARR
	{
		TIM2->SR &= ~TIM_SR_UIF;
	}
//	else if(TIM2->SR & TIM_SR_CC1IF) // from CCR1
//	{
//		TIM2->SR &= ~TIM_SR_CC1IF;
//	}

	// turn on ISR flag
	ISR_flag = 1;

//	GPIOC->ODR ^= (1 << GPIO_ODR_OD8_Pos);
}


// tests the DAC
void test_DAC()
{
	// voltage levels
	uint16_t v_low = DAC_volt_conv(1000);
	uint16_t v_high = DAC_volt_conv(3000);

	while (1)
	{
		for(int i = 0; i < 4; i++)
		{
			if(i != 0)
			{
				DAC_write(v_low);
			}
			else
			{
				DAC_write(v_high);
			}
		} // end for
	} // end while
}

// configures the system clock to be 32MHz
void sysclk_init()
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	*/
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	* in the RCC_OscInitTypeDef structure.
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
	//RCC_OscInitStruct.MSIState = RCC_MSI_ON;  //datasheet says NOT to turn on the MSI then change the frequency.
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_10;
	/* from stm32l4xx_hal_rcc.h
	#define RCC_MSIRANGE_0                 MSI = 100 KHz
	#define RCC_MSIRANGE_1                 MSI = 200 KHz
	#define RCC_MSIRANGE_2                 MSI = 400 KHz
	#define RCC_MSIRANGE_3                 MSI = 800 KHz
	#define RCC_MSIRANGE_4                 MSI = 1 MHz
	#define RCC_MSIRANGE_5                 MSI = 2 MHz
	#define RCC_MSIRANGE_6                 MSI = 4 MHz
	#define RCC_MSIRANGE_7                 MSI = 8 MHz
	#define RCC_MSIRANGE_8                 MSI = 16 MHz
	#define RCC_MSIRANGE_9                 MSI = 24 MHz
	#define RCC_MSIRANGE_10                MSI = 32 MHz
	#define RCC_MSIRANGE_11                MSI = 48 MHz   dont use this one*/
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;  //datasheet says NOT to turn on the MSI then change the frequency.
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);
	/** Initializes the CPU, AHB and APB buses clocks
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
						  	  	  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);
}

/*
 * Copyright 2018 Embedded Microprocessor Benchmark Consortium (EEMBC)
 * Copyright 2019 University of Rochester
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Original Author: Shay Gal-on
 * Ported to STM32L475 ARMv7-M board by: Zhuojia Shen
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "coremark.h"

#include "stm32l4xx.h"
#include "stm32l475e_iot01.h"

#if defined(VALIDATION_RUN)
volatile ee_s32 seed1_volatile = 0x3415;
volatile ee_s32 seed2_volatile = 0x3415;
volatile ee_s32 seed3_volatile = 0x66;
#elif defined(PERFORMANCE_RUN)
volatile ee_s32 seed1_volatile = 0x0;
volatile ee_s32 seed2_volatile = 0x0;
volatile ee_s32 seed3_volatile = 0x66;
#elif defined(PROFILE_RUN)
volatile ee_s32 seed1_volatile = 0x8;
volatile ee_s32 seed2_volatile = 0x8;
volatile ee_s32 seed3_volatile = 0x8;
#endif
volatile ee_s32 seed4_volatile = ITERATIONS;
volatile ee_s32 seed5_volatile = 0;

/*
 * Porting : Timing functions
 *
 * How to capture time and convert to seconds must be ported to whatever is
 * supported by the platform.  e.g. Read value from on board RTC, read value
 * from cpu clock cycles performance counter etc.  Sample implementation for
 * standard time.h and windows.h definitions included.
 */

/*
 * Define : TIMER_RES_DIVIDER
 *
 * Divider to trade off timer resolution and total time that can be measured.
 *
 * Use lower values to increase resolution, but make sure that overflow does
 * not occur.  If there are issues with the return value overflowing, increase
 * this value.
 */
#define NSECS_PER_SEC 1000
#define CORETIMETYPE uint32_t
#define GETMYTIME(_t) (*_t = HAL_GetTick())
#define MYTIMEDIFF(fin, ini) ((fin) - (ini))
#define TIMER_RES_DIVIDER 1
#define SAMPLE_TIME_IMPLEMENTATION 1
#define EE_TICKS_PER_SEC (NSECS_PER_SEC / TIMER_RES_DIVIDER)

/* Define Host specific (POSIX), or target specific global time variables. */
static CORETIMETYPE start_time_val, stop_time_val;

/*
 * Function : start_time
 *
 * This function will be called right before starting the timed portion of the
 * benchmark.
 *
 * Implementation may be capturing a system timer (as implemented in the
 * example code) or zeroing some system parameters - e.g. setting the cpu
 * clocks cycles to 0.
 */
void start_time(void)
{
	GETMYTIME(&start_time_val);
}

/*
 * Function : stop_time
 *
 * This function will be called right after ending the timed portion of the
 * benchmark.
 *
 * Implementation may be capturing a system timer (as implemented in the
 * example code) or other system parameters - e.g. reading the current value of
 * cpu cycles counter.
 */
void stop_time(void)
{
	GETMYTIME(&stop_time_val);
}

/*
 * Function : get_time
 *
 * Return an abstract "ticks" number that signifies time on the system.
 *
 * Actual value returned may be cpu cycles, milliseconds or any other value, as
 * long as it can be converted to seconds by <time_in_secs>.  This methodology
 * is taken to accomodate any hardware or simulated platform.  The sample
 * implementation returns millisecs by default, and the resolution is
 * controlled by <TIMER_RES_DIVIDER>
 */
CORE_TICKS get_time(void)
{
	CORE_TICKS elapsed=(CORE_TICKS)(MYTIMEDIFF(stop_time_val, start_time_val));
	return elapsed;
}

/*
 * Function : time_in_secs
 *
 * Convert the value returned by get_time to seconds.
 *
 * The <secs_ret> type is used to accomodate systems with no support for
 * floating point.  Default implementation implemented by the EE_TICKS_PER_SEC
 * macro above.
 */
secs_ret time_in_secs(CORE_TICKS ticks)
{
	secs_ret retval=((secs_ret)ticks) / (secs_ret)EE_TICKS_PER_SEC;
	return retval;
}

ee_u32 default_num_contexts = 1;

//===========================================================================//
// Timer initialization code
//===========================================================================//

void Error_Handler(void)
{
}

/*
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/* Initializes the CPU, AHB and APB busses clocks */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 40;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
	  Error_Handler();
	}

	/* Initializes the CPU, AHB and APB busses clocks */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
				      RCC_CLOCKTYPE_SYSCLK |
				      RCC_CLOCKTYPE_PCLK1 |
				      RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}
	/* Configure the main internal regulator output voltage */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
		Error_Handler();
	}
}

//===========================================================================//
// Console initialization code
//===========================================================================//

static UART_HandleTypeDef xConsoleUart;

static void
Console_UART_Init(void)
{
	xConsoleUart.Instance = USART1;
	xConsoleUart.Init.BaudRate = 115200;
	xConsoleUart.Init.WordLength = UART_WORDLENGTH_8B;
	xConsoleUart.Init.StopBits = UART_STOPBITS_1;
	xConsoleUart.Init.Parity = UART_PARITY_NONE;
	xConsoleUart.Init.Mode = UART_MODE_TX_RX;
	xConsoleUart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	xConsoleUart.Init.OverSampling = UART_OVERSAMPLING_16;
	xConsoleUart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	xConsoleUart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	BSP_COM_Init(0, &xConsoleUart);
}

void
vMainUARTPrintString(char * pcString)
{
	const uint32_t ulTimeout = 3000UL;

	HAL_UART_Transmit(&xConsoleUart, (uint8_t *) pcString,
			  strlen(pcString), ulTimeout);
}

/* From stm32cubel4's UART_printf example */
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */

/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
	/*
	 * Place your implementation of fputc here, e.g. write a character to
	 * the USART1 and Loop until the end of transmission.
	 */
	HAL_UART_Transmit(&xConsoleUart, (uint8_t *)&ch, 1, 0xFFFF);

	return ch;
}

//===========================================================================//
// MPU initialization code
//===========================================================================//

/* Defined in mpu.c */
void initMPU(void);

//===========================================================================//
// Portable code
//===========================================================================//

/*
 * Function : portable_init
 *
 * Target specific initialization code.
 * Test for some common mistakes.
 */
void portable_init(core_portable *p, int *argc, char *argv[])
{
	initMPU();
	SystemClock_Config();
	Console_UART_Init();

	if (sizeof(ee_ptr_int) != sizeof(ee_u8 *)) {
		ee_printf("ERROR! Please define ee_ptr_int to a type that holds a pointer!\n");
	}
	if (sizeof(ee_u32) != 4) {
		ee_printf("ERROR! Please define ee_u32 to a 32b unsigned type!\n");
	}
	p->portable_id=1;
}

/*
 * Function : portable_fini
 *
 * Target specific final code.
 */
void portable_fini(core_portable *p)
{
	p->portable_id=0;
}

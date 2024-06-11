/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define SHORT_LOWER 500
#define SHORT_UPPER 1500
#define LONG_LOWER 1500
#define LONG_UPPER 2500
#define CAPTURED_SIZE 16		// message size including single startbit
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */
uint8_t PDLC_intensity = 0; // varies between 0 and 60
uint16_t PDLC_setpoint = 0;
uint16_t PDLC_realpoint = 0;

uint32_t debug_value = 0;

struct Receiver {
//	uint32_t data[CAPTURED_SIZE];
	uint16_t data;
	uint32_t time_start;		// 0
	uint32_t time_stop;			// 0
	uint8_t first_interrupt;	// 1
	uint32_t captured[64];
	uint8_t position;			// 0
	uint8_t bit_count;			// 0
	uint32_t state;				// 0
	uint32_t start_state;		// 0

	uint8_t timeout_counter;	// 0
} Receiver1, Receiver2, Receiver3, Receiver4;

uint8_t flags = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void IR_Timeout(struct Receiver *Rec) {	// increments IR Receivers timeout counter and checks if timeout has reached threshold
	if (Rec->timeout_counter > TIMEOUT_THRESHOLD) {// checks if counter of timeout has reached threshold
		Rec->data = 0;					// resets data of receiver
	} else {
		Rec->timeout_counter++;		// increments counter of timeout
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//	TIM3->CCR3 = (Receiver4.data >> 1) * 40;// !WIP! updates dutycycle of PDLC PWM from received data (only receiver4 for now)

	IR_Timeout(&Receiver1);	// handles calcs and compares for IR timeout of receiver
	IR_Timeout(&Receiver2);
	IR_Timeout(&Receiver3);
	IR_Timeout(&Receiver4);

	uint8_t PDLC_candidate = Receiver1.data >> 1;	// writes data to PDLC_Candidate
	if(Receiver2.data > PDLC_candidate && Receiver2.data < 502) {			// checks if data Receiver2 is greater than PDLC_candidate
		PDLC_candidate = Receiver2.data >> 1;		// updates PDLC_candidate
	}
	if(Receiver3.data > PDLC_candidate && Receiver3.data < 502) {			// checks if data Receiver3 is greater than PDLC_candidate
		PDLC_candidate = Receiver3.data >> 1;		// updates PDLC_candidate
	}
	if(Receiver4.data > PDLC_candidate && Receiver4.data < 502) {			// checks if data Receiver4 is greater than PDLC_candidate
		PDLC_candidate = Receiver4.data >> 1;		// updates PDLC_candidate
	}

	PDLC_setpoint = PDLC_candidate * 40;			// updates PDLC_setpoint with 40 times the PDLC_candidate

	debug_value = (1.0-exp((double)((PDLC_setpoint/10000.0)*-2.1)))*10000;
	PDLC_setpoint = debug_value;

	if (PDLC_realpoint < PDLC_setpoint){			// checks if PDLC_realpoint is less than PDLC_setpoint
			PDLC_realpoint+= FADEIN_SPEED;				// increments PDLC_realpoint with FADEIN_SPEED
		} else if (PDLC_realpoint > PDLC_setpoint){		// checks if PDLC_realpoint is greater than PDLC_setpoint
			PDLC_realpoint-= FADEIN_SPEED;				// decrements PDLC_realpoint with FADEIN_SPEED
		}
	TIM3->CCR3 = PDLC_realpoint;					// sets timer for new dutycycle
}		// linerisation:

void IR_Receive_Handler(struct Receiver *Rec) {
	int elapsed = 0;
	if (Rec->first_interrupt == 1) {	// only runs once
		Rec->time_start = TIM1->CNT;	// saves first start time
		Rec->first_interrupt = 0;
	} else {
		Rec->time_stop = TIM1->CNT;	// saves stop time
		if (Rec->time_start > Rec->time_stop) {	// checks if overflowed
			elapsed = (0xFFFF - Rec->time_start) + Rec->time_stop;// calcs elapsed time
		} else {
			elapsed = Rec->time_stop - Rec->time_start;	// calcs elapsed time
		}
		Rec->time_start = Rec->time_stop;// saves starttime to stop time for next interrupt
	}
	Rec->captured[Rec->position] = elapsed;	// output buffer for data

	if (Rec->start_state) {
		switch (Rec->state) {
		case 0:
			if (elapsed > SHORT_LOWER && elapsed < SHORT_UPPER) {// checks if elapsed short
				Rec->state = 1;
			} else {	// unexpected behaviour : exit statemachine and reset
//								Rec->state = 0;		// NOTE : if commented code works, if not it enters for some reason
//								Rec->bit_count = 0;
//								Rec->start_state = 0;
			}
			break;
		case 1:
			if (elapsed > SHORT_LOWER && elapsed < SHORT_UPPER) {// checks if elapsed short
				Rec->state = 2;
			} else if (elapsed > LONG_LOWER && elapsed < LONG_UPPER) {// checks if elapsed long
				Rec->state = 3;
			} else {	// unexpected behaviour : exit statemachine and reset
				Rec->state = 0;	// NOTE : if commented code works, if not it enters for some reason
				Rec->bit_count = 0;
				Rec->start_state = 0;
				break;
			}
			Rec->data |= 1 << Rec->bit_count;	// write to data
			Rec->bit_count++;// increment the count of the bits already received
			if (Rec->bit_count == CAPTURED_SIZE) {
				Rec->state = 0;
				Rec->bit_count = 0;
				Rec->start_state = 0;
				Rec->timeout_counter = 0;
			}
//			if (Rec->bit_count > CAPTURED_SIZE) {// unexpected behaviour : exit statemachine and reset
//				Rec->state = 0;
//				Rec->bit_count = 0;
//				Rec->start_state = 0;
//			}		// commented out because this case will not happen
			break;
		case 2:
			if (elapsed > SHORT_LOWER && elapsed < SHORT_UPPER) {// checks if elapsed short
				Rec->state = 1;
			} else {	// unexpected behaviour : exit statemachine and reset
				Rec->state = 0;
				Rec->bit_count = 0;
				Rec->start_state = 0;
			}
			break;
		case 3:
			if (elapsed > SHORT_LOWER && elapsed < SHORT_UPPER) {// checks if elapsed short
				Rec->state = 4;
			} else if (elapsed > LONG_LOWER && elapsed < LONG_UPPER) {// checks if elapsed long
				Rec->state = 1;
			} else {	// unexpected behaviour : exit statemachine and reset
				Rec->state = 0;
				Rec->bit_count = 0;
				Rec->start_state = 0;
				break;
			}
			Rec->data &= ~(1 << Rec->bit_count);	// write to data
			Rec->bit_count++;// increment the count of the bits already received
			if (Rec->bit_count == CAPTURED_SIZE) {
				Rec->state = 0;
				Rec->bit_count = 0;
				Rec->start_state = 0;
				Rec->timeout_counter = 0;
			}
//			if (Rec->bit_count > CAPTURED_SIZE) {// unexpected behaviour : exit statemachine and reset
//				Rec->state = 0;
//				Rec->bit_count = 0;
//				Rec->start_state = 0;
//			}		// commented out because this case will not happen
			break;
		case 4:
			if (elapsed > SHORT_LOWER && elapsed < SHORT_UPPER) {// checks if elapsed short
				Rec->state = 3;
			} else {	// unexpected behaviour : exit statemachine and reset
				Rec->state = 0;
				Rec->bit_count = 0;
				Rec->start_state = 0;
			}
			break;
		}

	}

	if (elapsed > 2500) {// checks if "breakfield" is detected and enables the statemachine
		Rec->start_state = 1;
	}

	Rec->position++;	// increments position of buffer
	if (Rec->position > 63) {// handles overflow of buffer, 63 is determined by size captured array
		Rec->position = 0;	// makes buffer circular
	}
}

void init_IR_receiver(struct Receiver *Rec_init) {// sets standard values for structs
	Rec_init->time_start = 0;			// 0
	Rec_init->time_stop = 0;			// 0
	Rec_init->first_interrupt = 1;		// 1
	Rec_init->position = 0;				// 0
	Rec_init->bit_count = 0;			// 0
	Rec_init->state = 0;				// 0
	Rec_init->start_state = 0;			// 0
	Rec_init->timeout_counter = 0;		// 0
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */
	__HAL_RCC_TIM1_CLK_ENABLE();	// does not work without it
	TIM1->CR1 = TIM_CR1_CEN;	// does not work without it

	init_IR_receiver(&Receiver1);
	init_IR_receiver(&Receiver2);
	init_IR_receiver(&Receiver3);
	init_IR_receiver(&Receiver4);
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_TIM1_Init();
	MX_TIM5_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	/* USER CODE BEGIN 2 */
	if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)// error handler for PDLC PWM
			{
		/* Starting Error */
		Error_Handler();
	}

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

	if (HAL_TIM_Base_Start_IT(&htim4) != HAL_OK) {
		/* Starting Error */
		Error_Handler();
	}
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

//		TIM3->CCR3 = PDLC_intensity * 40;
//		// code for looping intensity from 0 to 250
//		if (PDLC_intensity > 250) {	// checks if PDLC_intensity is greater than 250
//			PDLC_intensity = 0;		// resets PDLC_intensity
//		} else {
//			PDLC_intensity++;		// increments PDLC_intensity
//		}
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 16 - 1;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 16 - 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 10000 - 1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	__HAL_TIM_DISABLE_OCxPRELOAD(&htim3, TIM_CHANNEL_3);
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 16;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 10000;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void) {

	/* USER CODE BEGIN TIM5_Init 0 */

	/* USER CODE END TIM5_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM5_Init 1 */

	/* USER CODE END TIM5_Init 1 */
	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 0;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 4294967295;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim5) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM5_Init 2 */

	/* USER CODE END TIM5_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pins : PA8 PA9 PA10 PA11 */
	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	switch (GPIO_Pin) {
	case GPIO_PIN_8:	// case for pin 8
		IR_Receive_Handler(&Receiver1);	// function to handle the IR data
		break;
	case GPIO_PIN_9:	// case for pin 5 different for PCB
		IR_Receive_Handler(&Receiver2);	// function to handle the IR data
		break;
	case GPIO_PIN_10:	// case for pin 10
		IR_Receive_Handler(&Receiver3);	// function to handle the IR data
		break;
	case GPIO_PIN_11:	// case for pin 6 different for PCB
		IR_Receive_Handler(&Receiver4);	// function to handle the IR data
		break;
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

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
#define MESSAGE_SIZE 8		// message size including single startbit
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
struct Receiver {
	uint32_t data[MESSAGE_SIZE];
	uint32_t time_start;		// 0
	uint32_t time_stop;			// 0
	uint8_t first_interrupt;	// 1
	uint32_t captured[32];
	uint8_t position;			// 0
	uint8_t bit_count;			// 0
	uint32_t state;				// 0
	uint32_t start_state;		// 0
} Receiver1, Receiver2, Receiver3, Receiver4;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void IR_Receive_Handler(struct Receiver* Rec) {
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
		Rec->time_start = Rec->time_stop;	// saves starttime to stop time for next interrupt
	}
	Rec->captured[Rec->position] = elapsed;	// output buffer for data

	if (Rec->start_state) {
		switch (Rec->state) {
		case 0:
			if (elapsed > SHORT_LOWER && elapsed < SHORT_UPPER) {// checks if elapsed short
				Rec->state = 1;
			} else {	// unexpected behaviour : exit statemachine and reset
				//				state = 0;		// NOTE : if commented code works, if not it enters for some reason
				//				bit_count = 0;
				//				start_state = 0;
			}
			break;
		case 1:
			if (elapsed > SHORT_LOWER && elapsed < SHORT_UPPER) {// checks if elapsed short
				Rec->state = 2;
			} else if (elapsed > LONG_LOWER && elapsed < LONG_UPPER) {// checks if elapsed long
				Rec->state = 3;
			} else {	// unexpected behaviour : exit statemachine and reset
				//				state = 0;		// NOTE : if commented code works, if not it enters for some reason
				//				bit_count = 0;
				//				start_state = 0;
				break;
			}
			Rec->data[Rec->bit_count] = 1;	// write to data
			Rec->bit_count++;	// increment the count of the bits already received
			if (Rec->bit_count > MESSAGE_SIZE) {// unexpected behaviour : exit statemachine and reset
				Rec->state = 0;
				Rec->bit_count = 0;
				Rec->start_state = 0;
			}
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
			Rec->data[Rec->bit_count] = 0;	// write to data
			Rec->bit_count++;	// increment the count of the bits already received
			if (Rec->bit_count > MESSAGE_SIZE) {// unexpected behaviour : exit statemachine and reset
				Rec->state = 0;
				Rec->bit_count = 0;
				Rec->start_state = 0;
			}
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
	if (Rec->position > 31) {
		Rec->position = 0;	// makes buffer circular
	}
}

void init_IR_receiver(struct Receiver* Rec_init){	// sets standard values for structs
	Rec_init->time_start = 0;			// 0
	Rec_init->time_stop = 0;			// 0
	Rec_init->first_interrupt = 1;		// 1
	Rec_init->position = 0;				// 0
	Rec_init->bit_count = 0;			// 0
	Rec_init->state = 0;				// 0
	Rec_init->start_state = 0;			// 0
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
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
  /* USER CODE BEGIN 2 */
	__HAL_RCC_TIM1_CLK_ENABLE();
	TIM1->PSC = HAL_RCC_GetPCLK1Freq() / 1000000 - 1;
	TIM1->ARR = 0xFFFFFFFF;
	TIM1->CR1 = TIM_CR1_CEN;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pins : PA5 PA6 PA8 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_10;
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
	case GPIO_PIN_5:	// case for pin 5 different for PCB
		IR_Receive_Handler(&Receiver2);	// function to handle the IR data
		break;
	case GPIO_PIN_10:	// case for pin 10
		IR_Receive_Handler(&Receiver3);	// function to handle the IR data
		break;
	case GPIO_PIN_6:	// case for pin 6 different for PCB
		IR_Receive_Handler(&Receiver4);	// function to handle the IR data
		break;
	}
//	IR_Receive_1();
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
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

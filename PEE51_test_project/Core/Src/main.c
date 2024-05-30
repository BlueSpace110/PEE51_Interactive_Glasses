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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "fonts.h"
#include "stdbool.h"
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// message array is the array that is used to output the manchester encoded bits
// this message array is thus twice as big as the amount of data sent
int16_t data_message = 0; // data to be transmitted, first lsb need to be1 always
uint32_t transmit_message = 0b00000000000000000000000000000001; // message to be transmitted, first two least significant bits need to be 0x01 always
int count = 0;		// count used for keeping track of amount of times IR interrupt is entered

uint16_t RC_counter = 0;
uint16_t time_passed = 0;
bool time_reached = false;

//Voor menu en settings bijhouden
int counter = 0;
int counter_old = 0;
static int menu_position = 0;
char str[10];
bool left;
bool right;
static bool setting_isbron = true;
static uint8_t setting_intensiteit = 0;
uint16_t setting_timer = 0;
uint8_t setting_timer_deel1 = 0;
uint8_t setting_timer_deel2 = 0;
static int seconds = 0;
bool save_data = 0;

enum size {
	small = 0, big = 1
};

// Intructies voor EEPROM ()

const uint8_t EEPROM_READ = 0b00000011;	//READ
const uint8_t EEPROM_WRITE = 0b00000010;	//WRITE
const uint8_t EEPROM_WRDI = 0b00000100;	//WRITE DISABLE
const uint8_t EEPROM_WREN = 0b00000110;	//WRITE ENABLE
const uint8_t EEPROM_RDSR = 0b00000101;	//READ STATUS REGISTER
const uint8_t EEPROM_WRSR = 0b00000001;	//WRITE STATUS REGISTER
const uint8_t MODE = 0b00000001;	//SETTING MODE OPSLAG LOCATIE
const uint8_t INTENSITEIT = 0b00000010;	//SETTING INTENSITEIT OPSLAG LOCATIE
const uint8_t DISPERSIONTIME1 = 0b00000011;	//SETTINGS DISPERSIONTIME DEEL 1 OPSLAG LOCATIE
const uint8_t DISPERSIONTIME2 = 0b00000100;	//SETTINGS DISPERSIONTIME DEEL 2 OPSLAG LOCATIE

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void user_pwm_setvalue(uint16_t value)
{
    TIM_OC_InitTypeDef sConfigOC;

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = value;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}

void data_to_transmit() {
	for (int i = 0; i < 16; i++)
	{
		if (data_message & (1 << i)) {
			transmit_message |= (1 << (i * 2));
			transmit_message &= ~(1 << (i * 2 + 1));
		} else {
			transmit_message &= ~(1 << (i * 2));
			transmit_message |= (1 << (i * 2 + 1));
		}
	}
}

void print_time(enum size size) {
	if (size == big) {
		ssd1306_SetCursor(0, 20);
		if (setting_timer / 3600 > 0) {
			sprintf(str, "%dh", setting_timer / 3600);
			ssd1306_WriteString(str, Font_16x26, White);
		} else if (setting_timer / 60 > 0) {
			sprintf(str, "%dm %ds", setting_timer / 60, setting_timer % 60);
			ssd1306_WriteString(str, Font_16x26, White);
		} else {
			sprintf(str, "%ds", setting_timer % 60);
			ssd1306_WriteString(str, Font_16x26, White);
		}

	} else {

		if (setting_timer <= 0) {
		}
		if (setting_timer / 3600 > 0) {
			sprintf(str, "%dh", setting_timer / 3600);
			ssd1306_WriteString(str, Font_7x10, White);
		} else if (setting_timer / 60 > 0) {
			sprintf(str, "%dm %ds", setting_timer / 60, setting_timer % 60);
			ssd1306_WriteString(str, Font_7x10, White);
		} else {
			sprintf(str, "%ds", setting_timer % 60);
			ssd1306_WriteString(str, Font_7x10, White);
		}
	}
}

void save_settings(uint8_t *address, uint8_t *value) {
	HAL_GPIO_WritePin(GPIOB, EEPROM_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &EEPROM_WREN, 1, 100);
	HAL_GPIO_WritePin(GPIOB, EEPROM_CS_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOB, EEPROM_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &EEPROM_WRITE, 1, 100);
	HAL_SPI_Transmit(&hspi1, address, 1, 100);
	HAL_SPI_Transmit(&hspi1, value, 1, 100);//Bron setting
	HAL_GPIO_WritePin(GPIOB, EEPROM_CS_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOB, EEPROM_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &EEPROM_WRDI, 1, 100);
	HAL_GPIO_WritePin(GPIOB, EEPROM_CS_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

	HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);

	if (ssd1306_Init(&hi2c1) != 0) {
		Error_Handler();
	}
	HAL_Delay(200);

	ssd1306_Fill(Black);
	ssd1306_UpdateScreen(&hi2c1);

	HAL_Delay(200);

	// Write data to local screenbuffer
	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString("Starting up", Font_7x10, White);

	ssd1306_SetCursor(0, 15);
	ssd1306_WriteString("Interactive", Font_7x10, White);

	ssd1306_SetCursor(0, 30);
	ssd1306_WriteString("Glasses", Font_7x10, White);

	ssd1306_SetCursor(0, 45);
	ssd1306_WriteString("Software V1.2", Font_7x10, White);
	// Copy all data from local screenbuffer to the screen
	ssd1306_UpdateScreen(&hi2c1);

	HAL_Delay(2000);

	uint8_t status = 0;
	uint8_t SETTINGS1 = 0;
	uint8_t SETTINGS2 = 0;
	uint8_t SETTINGS3 = 0;
	uint8_t SETTINGS4 = 0;
	uint16_t SETTINGS5 = 0;
//int test2 = 0;

	HAL_GPIO_WritePin(GPIOB, EEPROM_WP_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, EEPROM_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &EEPROM_WREN, 1, 100);
	HAL_GPIO_WritePin(GPIOB, EEPROM_CS_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOB, EEPROM_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &EEPROM_RDSR, 1, 100);
	HAL_SPI_Receive(&hspi1, &status, 1, 100);
	HAL_GPIO_WritePin(GPIOB, EEPROM_CS_Pin, GPIO_PIN_SET);

	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOB, EEPROM_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &EEPROM_READ, 1, 100);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &MODE, 1, 100);
	HAL_SPI_Receive(&hspi1, &SETTINGS1, 1, 100);
	HAL_GPIO_WritePin(GPIOB, EEPROM_CS_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOB, EEPROM_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &EEPROM_READ, 1, 100);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &INTENSITEIT, 1, 100);
	HAL_SPI_Receive(&hspi1, &SETTINGS2, 1, 100);
	HAL_GPIO_WritePin(GPIOB, EEPROM_CS_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOB, EEPROM_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &EEPROM_READ, 1, 100);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &DISPERSIONTIME1, 1, 100);
	HAL_SPI_Receive(&hspi1, &SETTINGS3, 1, 100);
	HAL_GPIO_WritePin(GPIOB, EEPROM_CS_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOB, EEPROM_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &EEPROM_READ, 1, 100);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &DISPERSIONTIME2, 1, 100);
	HAL_SPI_Receive(&hspi1, &SETTINGS4, 1, 100);
	HAL_GPIO_WritePin(GPIOB, EEPROM_CS_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOB, EEPROM_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &EEPROM_WRDI, 1, 100);
	HAL_GPIO_WritePin(GPIOB, EEPROM_CS_Pin, GPIO_PIN_SET);
	SETTINGS5 = (SETTINGS3 | (SETTINGS4 << 8));
	if (SETTINGS2 != 0 || SETTINGS5 != 0) {
		setting_isbron = SETTINGS1;
		setting_intensiteit = SETTINGS2;
		setting_timer = SETTINGS5;
		menu_position = 4;
		EXTI->SWIER |= EXTI_SWIER_SWIER14;

	} else {
		ssd1306_Fill(Black);
		ssd1306_UpdateScreen(&hi2c1);
		ssd1306_SetCursor(0, 0);
		ssd1306_WriteString("Rotate to begin", Font_7x10, White);
		ssd1306_UpdateScreen(&hi2c1);

	}

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
		if (save_data == 1) {
			save_settings((uint8_t*) &MODE, (uint8_t*) &setting_isbron);
			save_settings((uint8_t*) &INTENSITEIT, (uint8_t*) &setting_intensiteit);
			save_settings((uint8_t*) &DISPERSIONTIME1, (uint8_t*) &setting_timer_deel1);
			save_settings((uint8_t*) &DISPERSIONTIME2, (uint8_t*) &setting_timer_deel2);
			save_data = 0;
		}

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

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	NVIC_DisableIRQ(TIM4_IRQn);
	TIM4->CR1 &= ~TIM_CR1_CEN;
	TIM4->CNT = 0;
	NVIC_EnableIRQ(TIM2_IRQn);

	switch (menu_position) {
	case 0:
		if (left == true && right == false) {
			setting_isbron = true;
			menu_position = 1;
		} else if (left == false && right == true) {
			setting_isbron = false;
			menu_position = 2;
		}

		break;
	case 1:
		menu_position = 2;

		break;
	case 2:
		menu_position = 3;

		break;
	case 3:
		menu_position = 4;
		break;

	}

	ssd1306_SetCursor(0, 0);
	ssd1306_Fill(Black);
	switch (menu_position) {
	case 1:
		ssd1306_WriteString("Intensity ", Font_7x10, White);
		ssd1306_SetCursor(0, 20);
		sprintf(str, "%d", setting_intensiteit);
		ssd1306_WriteString(str, Font_16x26, White);
		break;
	case 2:
		ssd1306_WriteString("Dispersion time ", Font_7x10, White);
		ssd1306_SetCursor(0, 20);
		sprintf(str, "%ds", seconds);
		ssd1306_WriteString(str, Font_16x26, White);

		break;
	case 3:
		ssd1306_WriteString("Dispersion time ", Font_7x10, White);
		ssd1306_SetCursor(0, 20);
		if (setting_timer / 3600 > 0) {
			sprintf(str, "%dh", setting_timer / 3600);
			ssd1306_WriteString(str, Font_16x26, White);
		} else if (setting_timer / 60 > 0) {
			sprintf(str, "%dm %ds", setting_timer / 60, setting_timer % 60);
			ssd1306_WriteString(str, Font_16x26, White);
		} else {
			sprintf(str, "%ds", setting_timer % 60);
			ssd1306_WriteString(str, Font_16x26, White);
		}
		break;
	case 4:
		if (setting_isbron == true) {
			ssd1306_WriteString("Setup: Source", Font_7x10, White);
			ssd1306_SetCursor(0, 15);
			ssd1306_WriteString("Intensity:", Font_7x10, White);
			ssd1306_SetCursor(69, 15);
			sprintf(str, "%d", setting_intensiteit / 50);
			ssd1306_WriteString(str, Font_7x10, White);
			ssd1306_SetCursor(0, 30);
			ssd1306_WriteString("Dispersion:", Font_7x10, White);
			ssd1306_SetCursor(78, 30);
			print_time(small);
			ssd1306_SetCursor(0, 45);
			ssd1306_WriteString("Rotate to darken", Font_7x10, White);
			setting_timer_deel1 = (uint8_t) (setting_timer);
			setting_timer_deel2 = (uint8_t) (setting_timer >> 8);
			save_data = 1;

		} else {
			ssd1306_WriteString("Setup: Node", Font_7x10, White);
			ssd1306_SetCursor(0, 15);
			ssd1306_WriteString("Dispersion:", Font_7x10, White);
			ssd1306_SetCursor(80, 15);
			print_time(small);
			ssd1306_SetCursor(0, 45);
			ssd1306_WriteString("Rotate to darken", Font_7x10, White);
			setting_timer_deel1 = (uint8_t) (setting_timer);
			setting_timer_deel2 = (uint8_t) (setting_timer >> 8);
			save_data = 1;
		}

		menu_position = 5; //Goto enable sender
		break;
	case 5:
		ssd1306_WriteString("Reset? press to", Font_7x10, White);
		ssd1306_SetCursor(0, 10);
		ssd1306_WriteString("confirm", Font_7x10, White);
		ssd1306_SetCursor(0, 30);
		ssd1306_WriteString("Rotate to darken", Font_7x10, White);
		menu_position = 6;
		break;
	case 6:
		ssd1306_WriteString("Reset confirmed!", Font_7x10, White);
		ssd1306_SetCursor(0, 20);
		ssd1306_WriteString("Rotate to begin", Font_7x10, White);
		setting_intensiteit = 0;
		setting_timer = 0;
		time_passed = 0;
		time_reached = false;
		menu_position = 0;
		break;

	}
	ssd1306_UpdateScreen(&hi2c1);

}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {

	counter = (__HAL_TIM_GET_COUNTER(htim) / 4);

	if (counter > counter_old) {

		left = true;
		right = false;

		ssd1306_SetCursor(0, 0);
		ssd1306_Fill(Black);
		switch (menu_position) {
		case 0:
			ssd1306_WriteString("Setup as", Font_7x10, White);
			ssd1306_SetCursor(0, 20);
			ssd1306_WriteString("Source", Font_16x26, White);
			break;
		case 1:
			ssd1306_WriteString("Intensity ", Font_7x10, White);
			if (setting_intensiteit <= 0) {
				setting_intensiteit = 0;
			} else {
				setting_intensiteit = setting_intensiteit - 50;
			}
			ssd1306_SetCursor(0, 20);
			sprintf(str, "%d", setting_intensiteit / 50);
			ssd1306_WriteString(str, Font_16x26, White);
			break;
		case 2:
			ssd1306_WriteString("Dispersion time ", Font_7x10, White);
			if (setting_timer <= 0) {
			} else {
				setting_timer = setting_timer - 300;
			}
			print_time(big);
			break;
		case 3:
			ssd1306_WriteString("Dispersion time ", Font_7x10, White);
			if (setting_timer <= 0) {
			} else {
				setting_timer = setting_timer - 30;
			}
			print_time(big);
			break;
		case 5:
			NVIC_EnableIRQ(TIM4_IRQn); // Enable sending interrupe
			TIM4->CR1 |= TIM_CR1_CEN; // Enable timer counter
			TIM4->CNT = 0;
			NVIC_DisableIRQ(TIM2_IRQn); // Disable Rotary encoder
			break;
		case 6:
			menu_position = 5;
			NVIC_EnableIRQ(TIM4_IRQn); // Enable sending interrupe
			TIM4->CR1 |= TIM_CR1_CEN; // Enable timer counter
			TIM4->CNT = 0;
			NVIC_DisableIRQ(TIM2_IRQn); // Disable Rotary encoder
			break;

		}
	} else if (counter < counter_old) {

		right = true;
		left = false;
		ssd1306_SetCursor(0, 0);
		ssd1306_Fill(Black);
		switch (menu_position) {
		case 0:
			ssd1306_WriteString("Setup as", Font_7x10, White);
			ssd1306_SetCursor(0, 20);
			ssd1306_WriteString("Node", Font_16x26, White);
			break;
		case 1:
			ssd1306_WriteString("Intensity ", Font_7x10, White);
			if (setting_intensiteit >= 250) {
				setting_intensiteit = 250;
			} else {
				setting_intensiteit = setting_intensiteit + 50;
			}
			ssd1306_SetCursor(0, 20);
			sprintf(str, "%d", setting_intensiteit / 50);
			ssd1306_WriteString(str, Font_16x26, White);
			break;
		case 2:
			ssd1306_WriteString("Dispersion time ", Font_7x10, White);
			if (setting_timer >= 3600) {
			} else {
				setting_timer = setting_timer + 300;
			}
			ssd1306_SetCursor(0, 20);
			print_time(big);
			break;
		case 3:
			ssd1306_WriteString("Dispersion time ", Font_7x10, White);
			if (setting_timer >= 3600) {
			} else {
				setting_timer = setting_timer + 30;
			}
			ssd1306_SetCursor(0, 20);
			print_time(big);
			break;
		case 5:
			NVIC_EnableIRQ(TIM4_IRQn); // Enable sending interrupe
			TIM4->CR1 |= TIM_CR1_CEN; // Enable timer counter
			TIM4->CNT = 0;
			NVIC_DisableIRQ(TIM2_IRQn); // Disable Rotary encoder
			break;
		case 6:
			menu_position = 5;
			NVIC_EnableIRQ(TIM4_IRQn); // Enable sending interrupe
			TIM4->CR1 |= TIM_CR1_CEN; // Enable timer counter
			TIM4->CNT = 0;
			NVIC_DisableIRQ(TIM2_IRQn); // Disable Rotary encoder
			break;

		}

	}
	counter_old = counter;
	ssd1306_UpdateScreen(&hi2c1);

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (setting_timer == 0 && !time_reached) {
		data_message = (setting_intensiteit << 1) | 1;
		time_reached = true;
	} else if (time_passed < setting_timer && !time_reached) {
		if (RC_counter == 0) {
			data_message = ((time_passed * setting_intensiteit / setting_timer) << 1) | 1;
			RC_counter = 1000;
			time_passed++;
		}
	} else {
		if (!time_reached) time_reached = true;
		data_message = (setting_intensiteit << 1) | 1;
	}

	if (count < TRANSMISSION_SIZE) {// checks if in first 16 iterations of the message loop
		if (transmit_message & (1 << count)) {// checks if each bit of message array is 1
			//HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);// start PWM to drive IR LED
			user_pwm_setvalue(52);
		} else {					// else each bit of message array is 0
			//HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);// stop PWM to drive IR LED
			user_pwm_setvalue(0);
		}
	} else if (count == TRANSMISSION_SIZE) {// checks if on 17th iteration of message loop
		//HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);	// stop PWM to drive IR LED
		user_pwm_setvalue(0);
		data_to_transmit();
	} else if (count > MESSAGE_DELAY) {	// checks if message loop exceeds message_delay so that message loop resets
		count = -1;	// resets count for message loop to -1 because next count is increased by 1 to 0
	}

	count++;

	if (RC_counter > 0) RC_counter--;
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

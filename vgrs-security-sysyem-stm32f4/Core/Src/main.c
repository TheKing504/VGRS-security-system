/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

#include "stm32f4xx_hal.h"
#include "stdio.h"

#include "rc522.h"

#include "keypad.h"

#include "ledbar.h"

#include "buttons_comb.h"

#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define LOCKED		0
#define UNLOCKED  	1

#define EXPIRE_RFID_LOCK_VALIDATION_TIME		6000
#define EXPIRE_KEYPAD_LOCK_VALIDATION_TIME		6000

#define KEYPAD_PASSWORD_LENGTH					9
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;


// variable RFID
int8_t rc522State = -1;
unsigned char cardID[5];
unsigned char validID_1[5] = {0x39, 0xE1, 0x19, 0xB3, 0x72};	// chip
unsigned char validID_2[5] = {0x26, 0x08, 0xD6, 0x12, 0xEA};	// card
int64_t rfidLastTimeValidated = -1;
int8_t rfidLockState = LOCKED;


// variable keypad
char passwordInput[KEYPAD_PASSWORD_LENGTH];
char validPassword_1[KEYPAD_PASSWORD_LENGTH] = "234567899";
char validPassword_2[KEYPAD_PASSWORD_LENGTH] = "998765432";
int64_t keypadLastTimeValidated = -1;
int8_t keypadLockState = LOCKED;
char c = '!';

// variable buttons comb
int8_t buttonsCombInput[NUM_OF_BB_SIZE] = {0, 0, 0, 0};
int8_t validButtonsComb[NUM_OF_BB_SIZE] = {1, 0, 1, 0};
int8_t buttonsCombLockState = LOCKED;

// variable PIR sensor
int8_t PIRSensorState = 0;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_SPI1_Init();
  MFRC522_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(1000);
  /* USER CODE END 2 */

  makeSound2Beeb();

  makeSound1Beeb();

  makeSound1Beeb();

  makeSound2Beeb();

  setLedbarTo(0);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  PIRSensorState = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);

	  /* USER CODE END WHILE */
	  readButtonCombLockState();
	  readKeypadLockState();
	  readRFIDLockState();

	  setCommunicationPins();
	  /* USER CODE BEGIN 3 */
  }
  /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

void setCommunicationPins()
{
	  if (rfidLockState == UNLOCKED)
	  {
		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, 1);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
	  } else {
		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, 0);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
	  }

	  if (keypadLockState == UNLOCKED)
	  {
		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, 1);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 1);
	  } else {
		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, 0);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);
	  }

	  if (buttonsCombLockState == UNLOCKED)
	  {
		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, 1);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 1);
	  } else {
		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, 0);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0);
	  }

	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, PIRSensorState);
}

void readButtonCombLockState()
{
	updateButtonsCombState(&buttonsCombInput);
	int8_t isValidCombState = 1;
	for (int i = 0; i < NUM_OF_BB_SIZE; i++)
	{
		if (buttonsCombInput[i] != validButtonsComb[i])
		{
			isValidCombState = 0;
			break;
		}
	}
	if (isValidCombState == 1)
	{
		buttonsCombLockState = UNLOCKED;
	} else
	{
		buttonsCombLockState = LOCKED;
	}
}

void readKeypadLockState()
{
	if (keypadLastTimeValidated != -1 && HAL_GetTick() - keypadLastTimeValidated <= EXPIRE_KEYPAD_LOCK_VALIDATION_TIME && keypadLockState == UNLOCKED)
	{
		return;
	} else {
		keypadLockState = LOCKED;
	}

	c = readKeypad();

	if (c == '#') {
		if (strcmp(passwordInput, validPassword_1) == 0 || strcmp(passwordInput, validPassword_2) == 0)
		{
			makeSound1Beeb();	// ok
			keypadLockState = UNLOCKED;
			keypadLastTimeValidated = HAL_GetTick();
		} else {
			makeSound2Beeb(); 	// wrong
			keypadLockState = LOCKED;
		}
		passwordInput[0] = '\0';
		setLedbarTo(0);
		return;
	}

	if (c == 'C') {
		// clear password
		passwordInput[0] = '\0';
		setLedbarTo(0);
		return;
	}

	if (strlen(passwordInput) == KEYPAD_PASSWORD_LENGTH)
	{
		return;
	}

	// nothing clicked
	if (c == '-') {
		return;
	}

	strncat(passwordInput, &c, 1);
	setLedbarTo(strlen(passwordInput));

}

void readRFIDLockState()
{
	if (rfidLastTimeValidated != -1 && HAL_GetTick() - rfidLastTimeValidated <= EXPIRE_RFID_LOCK_VALIDATION_TIME && rfidLockState == UNLOCKED)
	{
		return;
	}
	rc522State = MFRC522_Check(cardID);
	if (rc522State == MI_OK)
	{
		if (MFRC522_Compare(cardID, validID_1) == MI_OK || MFRC522_Compare(cardID, validID_2) == MI_OK)
		{
			makeSound1Beeb();	// ok
			rfidLockState = UNLOCKED;
			rfidLastTimeValidated = HAL_GetTick();
		} else
		{
			makeSound2Beeb();	// wrong card
			rfidLockState = LOCKED;
		}
	} else {
		// no card detected
		rfidLockState = LOCKED;
	}
	HAL_Delay(10);

}

void makeSound2Beeb()
{
	for(int i = 0; i < 40; i++)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
		HAL_Delay(5);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_Delay(2);
	}

	HAL_Delay(200);

	for(int i = 0; i < 40; i++)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
		HAL_Delay(5);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_Delay(2);
	}
}

void makeSound1Beeb()
{
	for(int i = 0; i < 100; i++)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
		HAL_Delay(5);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_Delay(2);
	}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
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

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pins : PA4 - CS PIN for RC522(SPI1)*/
  GPIO_InitStruct.Pin = CS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(CS_PORT, &GPIO_InitStruct);


  /*Configure GPIO pins : PD12 PD13 PD14 PD15 - BOARD LEDs*/
  GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);


  /*Configure GPIO pins for keypad  */
  GPIO_InitTypeDef keypadRows;
  keypadRows.Pin =  R1_PIN | R2_PIN | R3_PIN | R4_PIN;
  keypadRows.Mode = GPIO_MODE_OUTPUT_PP;
  keypadRows.Pull = GPIO_NOPULL;
  keypadRows.Speed = GPIO_SPEED_FREQ_LOW;

  HAL_GPIO_Init(R1_PORT, &keypadRows);

  GPIO_InitTypeDef keypadColumns;
  keypadColumns.Pin =  C1_PIN | C2_PIN | C3_PIN | C4_PIN;
  keypadColumns.Mode = GPIO_MODE_INPUT;
  keypadColumns.Pull = GPIO_PULLDOWN;

  HAL_GPIO_Init(C1_PORT,&keypadColumns);

  /*Configure GPIO pins for ledbar  */
  GPIO_InitTypeDef ledbar;
  ledbar.Pin =  L1_PIN | L2_PIN | L3_PIN | L4_PIN | L5_PIN | L6_PIN | L7_PIN | L8_PIN | L9_PIN | L10_PIN;
  ledbar.Mode = GPIO_MODE_OUTPUT_PP;
  ledbar.Pull = GPIO_NOPULL;
  ledbar.Speed = GPIO_SPEED_FREQ_LOW;

  HAL_GPIO_Init(LEDBAR_PORT, &ledbar);

  /*Configure GPIO pins for ledcomb  */
  GPIO_InitTypeDef ledcombButtons;
  ledcombButtons.Pin =  BB1_PIN | BB2_PIN | BB3_PIN | BB4_PIN;
  ledcombButtons.Mode = GPIO_MODE_INPUT;
  ledcombButtons.Pull = GPIO_PULLDOWN;

  HAL_GPIO_Init(BUTTONS_COMB_PORT, &ledcombButtons);

  GPIO_InitTypeDef ledcombLeds;
  ledcombLeds.Pin =  BL1_PIN | BL2_PIN | BL3_PIN | BL4_PIN;
  ledcombLeds.Mode = GPIO_MODE_OUTPUT_PP;
  ledcombLeds.Pull = GPIO_NOPULL;
  ledcombLeds.Speed = GPIO_SPEED_FREQ_LOW;

  HAL_GPIO_Init(BUTTONS_COMB_PORT, &ledcombLeds);

  /*Configure GPIO pin for PIR sensor  */
  GPIO_InitTypeDef pirsensor;
  pirsensor.Pin =  GPIO_PIN_0;
  pirsensor.Mode = GPIO_MODE_INPUT;
  pirsensor.Pull = GPIO_PULLDOWN;

  HAL_GPIO_Init(GPIOA, &pirsensor);


  /*Configure GPIO pin for buzzer  */
  GPIO_InitTypeDef buzzer;
  buzzer.Pin =  GPIO_PIN_8;
  buzzer.Mode = GPIO_MODE_OUTPUT_PP;
  buzzer.Pull = GPIO_NOPULL;
  buzzer.Speed = GPIO_SPEED_FREQ_LOW;

  HAL_GPIO_Init(GPIOA, &buzzer);

  /*Configure GPIO pin for communication with stm32f7  */
  GPIO_InitTypeDef stm32f7;
  stm32f7.Pin =  GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14;
  stm32f7.Mode = GPIO_MODE_OUTPUT_PP;
  stm32f7.Pull = GPIO_NOPULL;
  stm32f7.Speed = GPIO_SPEED_FREQ_LOW;

  HAL_GPIO_Init(GPIOE, &stm32f7);

}

/* USER CODE BEGIN 4 */

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
  while (1)
  {
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


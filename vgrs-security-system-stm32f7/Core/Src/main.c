/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f769i_discovery.h"
#include "stm32f769i_discovery_lcd.h"
#include "stm32f769i_discovery_ts.h"
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

/* Definitions for defaultTask */

osThreadId_t mainTaskHandle;
const osThreadAttr_t mainTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* USER CODE BEGIN PV */
uint8_t  lcd_status = LCD_OK;
uint32_t ts_status = TS_OK;
TS_StateTypeDef  TS_State = {0};


#define LOCKED 		0
#define UNLOCKED	1

uint8_t previousKeypadLockState = LOCKED;
uint8_t keypadLockState = LOCKED;
int64_t keypadLockStateLCDlastTimeUpdated = -1;

uint8_t previousRfidLockState = LOCKED;
uint8_t rfidLockState = LOCKED;
int64_t rfidLockStateLCDlastTimeUpdated = -1;


uint8_t previousbuttonsCombLockState = LOCKED;
uint8_t buttonsCombLockState = LOCKED;
int64_t buttonsCombStateLCDlastTimeUpdated = -1;

uint8_t previousPirSensorState = 0;
uint8_t pirSensorState = 0;
int64_t pirSensorStateLCDlastTimeUpdated = -1;

uint8_t mainLockState = LOCKED;
int64_t mainLockStateLastTimeUpdated = -1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void mainTask(void *argument);
void StartDefaultTask(void *argument);

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
  /* USER CODE BEGIN 2 */
  MX_GPIO_Init();

  BSP_LCD_Init();

  BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);
  BSP_LCD_Clear(LCD_COLOR_BLACK);

  ts_status = BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());
  while(ts_status != TS_OK);

  ts_status = BSP_TS_ITConfig();
  while(ts_status != TS_OK);

  drawFirmTitleOnLCD();

  drawCameraStateOnLCD();

  drawSecLevelsTitlesOnLCD();
  drawSecLevel1ValueOnLCD();

  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_RESET);	// close reley

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  mainTaskHandle = osThreadNew(mainTask, NULL, &mainTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void drawFirmTitleOnLCD()
{
	 BSP_LCD_SetFont(&Font24);
	 BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	 BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	 BSP_LCD_DisplayStringAtLine(1, " Advanced SECURITY SYSTEM - Kraljevo varovanje");
}

void drawSecLevelsTitlesOnLCD()
{
	 BSP_LCD_SetFont(&Font24);
	 BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGRAY);
	 BSP_LCD_SetBackColor(LCD_COLOR_BLACK);

	 BSP_LCD_DisplayStringAtLine(4, "              Security level 1:");
	 BSP_LCD_DisplayStringAtLine(9, "              Security level 2:");
	 BSP_LCD_DisplayStringAtLine(14, "              Security level 3:");


	 BSP_LCD_SetTextColor(LCD_COLOR_RED);
	 BSP_LCD_DisplayStringAtLine(6, "               NOT ACTIVATED");
	 BSP_LCD_DisplayStringAtLine(11, "               NOT ACTIVATED");
	 BSP_LCD_DisplayStringAtLine(16, "               NOT ACTIVATED");
}

// buttons combs
void drawSecLevel1ValueOnLCD()
{

	BSP_LCD_SetFont(&Font24);
	BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGRAY);
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK);

	BSP_LCD_ClearStringLine(6);
	if (buttonsCombLockState == LOCKED)
	{
		 BSP_LCD_SetTextColor(LCD_COLOR_RED);
		 BSP_LCD_DisplayStringAtLine(6, "                NOT ACTIVATED");
	} else
	{
		 BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
		 BSP_LCD_DisplayStringAtLine(6, "                 ACTIVATED");
	}
}

// rfid
void drawSecLevel2ValueOnLCD()
{
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGRAY);
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK);

	BSP_LCD_ClearStringLine(11);
	if (rfidLockState == LOCKED)
	{
		 BSP_LCD_SetTextColor(LCD_COLOR_RED);
		 BSP_LCD_DisplayStringAtLine(11, "                NOT ACTIVATED");
	} else
	{
		 BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
		 BSP_LCD_DisplayStringAtLine(11, "                 ACTIVATED");
	}
}

// keypad
void drawSecLevel3ValueOnLCD()
{
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGRAY);
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK);

	BSP_LCD_ClearStringLine(16);
	if (keypadLockState == LOCKED)
	{
		 BSP_LCD_SetTextColor(LCD_COLOR_RED);
		 BSP_LCD_DisplayStringAtLine(16, "                NOT ACTIVATED");
	} else
	{
		 BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
		 BSP_LCD_DisplayStringAtLine(16, "                 ACTIVATED");
	}
}

void drawCameraStateOnLCD()
{
	BSP_LCD_SetFont(&Font16);
	BSP_LCD_ClearStringLine(29);
	BSP_LCD_SetTextColor(LCD_COLOR_GRAY);
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK);

	if (pirSensorState == 1)
	{
		BSP_LCD_DisplayStringAtLine(29, " Camera is activated.");
	} else
	{
		BSP_LCD_DisplayStringAtLine(29, " Camera is not activated.");
	}
}

void drawMainLockedUnlocked()
{
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_ClearStringLine(6);
	BSP_LCD_ClearStringLine(11);
	BSP_LCD_ClearStringLine(16);

	BSP_LCD_ClearStringLine(4);
	BSP_LCD_ClearStringLine(9);
	BSP_LCD_ClearStringLine(14);

	BSP_LCD_SetTextColor(LCD_COLOR_ORANGE);
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK);

	BSP_LCD_DisplayStringAtLine(11, "            MAIN LOCK ACTIVATED");
}

void drawMainLockedLocked() {
	BSP_LCD_ClearStringLine(11);
	drawSecLevelsTitlesOnLCD();
	drawSecLevel1ValueOnLCD();
	drawSecLevel2ValueOnLCD();
	drawSecLevel3ValueOnLCD();
}

void MX_GPIO_Init(void)
{
	  GPIO_InitTypeDef GPIO_InitStruct;

	  /* GPIO Ports Clock Enable */
	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOC_CLK_ENABLE();
	  __HAL_RCC_GPIOF_CLK_ENABLE();

	  /*Configure GPIO pins*/
	  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_6;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  GPIO_InitStruct.Pin = GPIO_PIN_2;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  GPIO_InitStruct.Pin = GPIO_PIN_10;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	  GPIO_InitStruct.Pin = GPIO_PIN_8;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef  ret = HAL_OK;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is
	 clocked below the maximum system frequency, to update the voltage scaling value
	 regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  RCC_OscInitStruct.PLL.PLLR = 7;

  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
	while(1) { ; }
  }

  /* Activate the OverDrive to reach the 216 MHz Frequency */
  ret = HAL_PWREx_EnableOverDrive();
  if(ret != HAL_OK)
  {
	while(1) { ; }
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7);
  if(ret != HAL_OK)
  {
	while(1) { ; }
  }
}

/* USER CODE BEGIN 4 */

void mainTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  keypadLockState = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);
	  rfidLockState = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
	  buttonsCombLockState = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2);
	  pirSensorState = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_10);
	  HAL_Delay(5);


	  if (previousPirSensorState != pirSensorState && (pirSensorStateLCDlastTimeUpdated == -1 || HAL_GetTick() - pirSensorStateLCDlastTimeUpdated > 3000))
	  {
		  drawCameraStateOnLCD();
		  previousPirSensorState = pirSensorState;
		  pirSensorStateLCDlastTimeUpdated = HAL_GetTick();
	  }
	  if (previousbuttonsCombLockState != buttonsCombLockState && (buttonsCombStateLCDlastTimeUpdated == -1 || HAL_GetTick() - buttonsCombStateLCDlastTimeUpdated > 1000))
	  {
		  drawSecLevel1ValueOnLCD();
		  previousbuttonsCombLockState = buttonsCombLockState;
		  buttonsCombStateLCDlastTimeUpdated = HAL_GetTick();
	  }

	  if (previousRfidLockState != rfidLockState && (rfidLockStateLCDlastTimeUpdated == -1 || HAL_GetTick() - rfidLockStateLCDlastTimeUpdated > 100))
	  {
		  drawSecLevel2ValueOnLCD();
		  previousRfidLockState = rfidLockState;
		  rfidLockStateLCDlastTimeUpdated = HAL_GetTick();
	  }

	  if (previousKeypadLockState != keypadLockState && (keypadLockStateLCDlastTimeUpdated == -1 || HAL_GetTick() - keypadLockStateLCDlastTimeUpdated > 100))
	  {
		  drawSecLevel3ValueOnLCD();
		  previousKeypadLockState = keypadLockState;
		  keypadLockStateLCDlastTimeUpdated = HAL_GetTick();
	  }

	  if (buttonsCombLockState == UNLOCKED && rfidLockState == UNLOCKED && keypadLockState == UNLOCKED && mainLockState == LOCKED)
	  {
		  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_SET);	// Open reley
		  mainLockState = UNLOCKED;
		  mainLockStateLastTimeUpdated = HAL_GetTick();
		  drawMainLockedUnlocked();
	  } else
	  {
		  if (mainLockStateLastTimeUpdated != -1 && HAL_GetTick() - mainLockStateLastTimeUpdated < 30000)
		  {
			  // Wait - do nothing
		  } else if (mainLockState == UNLOCKED)
		  {
			  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_RESET);	// close reley
			  mainLockState = LOCKED;
			  drawMainLockedLocked();
		  }
	  }
  }
  /* USER CODE END 5 */
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    BSP_TS_GetState(&TS_State);
	if(TS_State.touchDetected > 0) {
		if (TS_State.touchY[0] > 30)
		BSP_LCD_DrawCircle(TS_State.touchX[0], TS_State.touchY[0], 30);
	}
    osDelay(20);
  }
  /* USER CODE END 5 */
}

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


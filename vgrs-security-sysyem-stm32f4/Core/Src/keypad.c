// Reference: https://controllerstech.com/use-4x4-keypad-with-stm32/

#include "stm32f4xx_hal.h"
#include "keypad.h"


char readKeypad (void)
{
	HAL_GPIO_WritePin (R1_PORT, R1_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin (R2_PORT, R2_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin (R3_PORT, R3_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin (R4_PORT, R4_PIN, GPIO_PIN_RESET);

	if (HAL_GPIO_ReadPin (C1_PORT, C1_PIN))
	{
		while (HAL_GPIO_ReadPin (C1_PORT, C1_PIN));
		return '1';
	}
	if (HAL_GPIO_ReadPin (C2_PORT, C2_PIN))
	{
		while (HAL_GPIO_ReadPin (C2_PORT, C2_PIN));
		return '2';
	}
	if (HAL_GPIO_ReadPin (C3_PORT, C3_PIN))
	{
		while (HAL_GPIO_ReadPin (C3_PORT, C3_PIN));
		return '3';
	}
	if (HAL_GPIO_ReadPin (C4_PORT, C4_PIN))
	{
		while (HAL_GPIO_ReadPin (C4_PORT, C4_PIN));
		return 'A';
	}

	HAL_GPIO_WritePin (R1_PORT, R1_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin (R2_PORT, R2_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin (R3_PORT, R3_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin (R4_PORT, R4_PIN, GPIO_PIN_RESET);

	if (HAL_GPIO_ReadPin (C1_PORT, C1_PIN))
	{
		while (HAL_GPIO_ReadPin (C1_PORT, C1_PIN));
		return '4';
	}
	if (HAL_GPIO_ReadPin (C2_PORT, C2_PIN))
	{
		while (HAL_GPIO_ReadPin (C2_PORT, C2_PIN));
		return '5';
	}
	if (HAL_GPIO_ReadPin (C3_PORT, C3_PIN))
	{
		while (HAL_GPIO_ReadPin (C3_PORT, C3_PIN));
		return '6';
	}
	if (HAL_GPIO_ReadPin (C4_PORT, C4_PIN))
	{
		while (HAL_GPIO_ReadPin (C4_PORT, C4_PIN));
		return 'B';
	}

	HAL_GPIO_WritePin (R1_PORT, R1_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin (R2_PORT, R2_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin (R3_PORT, R3_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin (R4_PORT, R4_PIN, GPIO_PIN_RESET);

	if (HAL_GPIO_ReadPin (C1_PORT, C1_PIN))
	{
		while (HAL_GPIO_ReadPin (C1_PORT, C1_PIN));
		return '7';
	}
	if (HAL_GPIO_ReadPin (C2_PORT, C2_PIN))
	{
		while (HAL_GPIO_ReadPin (C2_PORT, C2_PIN));
		return '8';
	}
	if (HAL_GPIO_ReadPin (C3_PORT, C3_PIN))
	{
		while (HAL_GPIO_ReadPin (C3_PORT, C3_PIN));
		return '9';
	}
	if (HAL_GPIO_ReadPin (C4_PORT, C4_PIN))
	{
		while (HAL_GPIO_ReadPin (C4_PORT, C4_PIN));
		return 'C';
	}

	HAL_GPIO_WritePin (R1_PORT, R1_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin (R2_PORT, R2_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin (R3_PORT, R3_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin (R4_PORT, R4_PIN, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin (C1_PORT, C1_PIN))
	{
		while (HAL_GPIO_ReadPin (C1_PORT, C1_PIN));
		return '*';
	}
	if (HAL_GPIO_ReadPin (C2_PORT, C2_PIN))
	{
		while (HAL_GPIO_ReadPin (C2_PORT, C2_PIN));
		return '0';
	}
	if (HAL_GPIO_ReadPin (C3_PORT, C3_PIN))
	{
		while (HAL_GPIO_ReadPin (C3_PORT, C3_PIN));
		return '#';
	}
	if (HAL_GPIO_ReadPin (C4_PORT, C4_PIN))
	{
		while (HAL_GPIO_ReadPin (C4_PORT, C4_PIN));
		return 'D';
	}

	return '-';
}

// Reference: http://www.learningaboutelectronics.com/Articles/10-segment-LED-bar-graph-circuit-with-an-arduino.php

#include "stm32f4xx_hal.h"
#include "ledbar.h"

void setLedbar(uint8_t conf[])
{
	HAL_GPIO_WritePin (LEDBAR_PORT, L1_PIN, conf[0] != 0);
	HAL_GPIO_WritePin (LEDBAR_PORT, L2_PIN, conf[1] != 0);
	HAL_GPIO_WritePin (LEDBAR_PORT, L3_PIN, conf[2] != 0);
	HAL_GPIO_WritePin (LEDBAR_PORT, L4_PIN, conf[3] != 0);
	HAL_GPIO_WritePin (LEDBAR_PORT, L5_PIN, conf[4] != 0);
	HAL_GPIO_WritePin (LEDBAR_PORT, L6_PIN, conf[5] != 0);
	HAL_GPIO_WritePin (LEDBAR_PORT, L7_PIN, conf[6] != 0);
	HAL_GPIO_WritePin (LEDBAR_PORT, L8_PIN, conf[7] != 0);
	HAL_GPIO_WritePin (LEDBAR_PORT, L9_PIN, conf[8] != 0);
	HAL_GPIO_WritePin (LEDBAR_PORT, L10_PIN, conf[9] != 0);
}

void setLedbarTo(int8_t to) {
	uint8_t conf[10];
	for (int i = 0; i < to; i++)
	{
		conf[i] = 1;
	}
	setLedbar(conf);
}

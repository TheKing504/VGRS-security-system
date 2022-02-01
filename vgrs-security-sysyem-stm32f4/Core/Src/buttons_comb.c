#include "stm32f4xx_hal.h"
#include "buttons_comb.h"

int64_t lastTimeUpdated = -1;
int8_t bb_led1State = 0;
int8_t bb_led2State = 0;
int8_t bb_led3State = 0;
int8_t bb_led4State = 0;

void updateButtonsCombState(uint8_t* buttonsCombState)
{
	if (lastTimeUpdated == -1)
	{
		lastTimeUpdated = HAL_GetTick();
	}

    if (HAL_GetTick() - lastTimeUpdated >= BB_BOUNCING_FACTOR)
    {
    	int8_t bc1 = HAL_GPIO_ReadPin(BUTTONS_COMB_PORT, BB1_PIN);
    	int8_t bc2 = HAL_GPIO_ReadPin(BUTTONS_COMB_PORT, BB2_PIN);
    	int8_t bc3 = HAL_GPIO_ReadPin(BUTTONS_COMB_PORT, BB3_PIN);
    	int8_t bc4 = HAL_GPIO_ReadPin(BUTTONS_COMB_PORT, BB4_PIN);

    	if (bc1 == GPIO_PIN_SET)
    	{
    		bb_led1State = toogleBBState(bb_led1State);
    	}

    	if (bc2 == GPIO_PIN_SET)
    	{
    		bb_led2State = toogleBBState(bb_led2State);
    	}

    	if (bc3 == GPIO_PIN_SET)
    	{
    		bb_led3State = toogleBBState(bb_led3State);
    	}

    	if (bc4 == GPIO_PIN_SET)
    	{
    		bb_led4State = toogleBBState(bb_led4State);
    	}

    	HAL_GPIO_WritePin(BUTTONS_COMB_PORT, BL1_PIN, bb_led1State);
    	HAL_GPIO_WritePin(BUTTONS_COMB_PORT, BL2_PIN, bb_led2State);
    	HAL_GPIO_WritePin(BUTTONS_COMB_PORT, BL3_PIN, bb_led3State);
    	HAL_GPIO_WritePin(BUTTONS_COMB_PORT, BL4_PIN, bb_led4State);

    	lastTimeUpdated = HAL_GetTick();
    }

	buttonsCombState[0] = bb_led1State;
	buttonsCombState[1] = bb_led2State;
	buttonsCombState[2] = bb_led3State;
	buttonsCombState[3] = bb_led4State;

}

int8_t toogleBBState(int8_t bb_ledState)
{
	if (bb_ledState == 0)
	{
		return (int8_t)1;
	}
	return (int8_t)0;
}

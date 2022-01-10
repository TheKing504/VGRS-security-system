
#define BUTTONS_COMB_PORT GPIOD

// BB - Button comb button
// BL - Button comb LED
#define BB1_PIN GPIO_PIN_0
#define BB2_PIN GPIO_PIN_1
#define BB3_PIN GPIO_PIN_2
#define BB4_PIN GPIO_PIN_3

#define BL1_PIN GPIO_PIN_9
#define BL2_PIN GPIO_PIN_10
#define BL3_PIN GPIO_PIN_11
#define BL4_PIN GPIO_PIN_12

#define NUM_OF_BB_SIZE			4
#define BB_BOUNCING_FACTOR 		200

// WARNING - when changing keypad pins configuration fix MX_GPIO_Init in the main.c

int8_t toogleBBState(int8_t bb_ledState);
void updateButtonsCombState(uint8_t* buttonsCombState);


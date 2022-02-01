// Reference: https://controllerstech.com/use-4x4-keypad-with-stm32/

#define R1_PORT GPIOB
#define R1_PIN GPIO_PIN_7

#define R2_PORT GPIOB
#define R2_PIN GPIO_PIN_6

#define R3_PORT GPIOB
#define R3_PIN GPIO_PIN_5

#define R4_PORT GPIOB
#define R4_PIN GPIO_PIN_4

#define C1_PORT GPIOB
#define C1_PIN GPIO_PIN_8

#define C2_PORT GPIOB
#define C2_PIN GPIO_PIN_2

#define C3_PORT GPIOB
#define C3_PIN GPIO_PIN_1

#define C4_PORT GPIOB
#define C4_PIN GPIO_PIN_0

// WARNING - when changing keypad pins configuration fix MX_GPIO_Init in the main.c
// Problem is with ports

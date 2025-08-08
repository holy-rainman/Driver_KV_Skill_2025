#include "stm32l0xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>

//=================================================================== I/O Pins AML
#define LOW 		GPIO_PIN_RESET
#define HIGH 		GPIO_PIN_SET

#define dataS(x)	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, x? HIGH:LOW)
#define shCLK(x) 	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, x? HIGH:LOW)
#define stCLK(x) 	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, x? HIGH:LOW)

#define BUZ(x) 		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 ,x? LOW:HIGH)

#define LED1(x) 	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,x? LOW:HIGH)
#define LED2(x) 	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,x? LOW:HIGH)
#define LED3(x) 	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,x? LOW:HIGH)
#define LED4(x) 	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,x? LOW:HIGH)

#define S1 			HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)
#define S2 			HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4)
#define S3 			HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)

//=================================================================== Function Prototype
uint8_t getPB(void);
void LEDs(uint8_t decimal);
void displaySegment(void);
void multiplexSegment(uint16_t num4digit);
void multiplexChar(const char* text);
void multiplexClear();

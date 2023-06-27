/*
 * GY-25.c
 *
 *  Created on: Jun 21, 2023
 *      Author: 98912
 */
#include "main.h"

uint8_t GY_A5[] = {0xA5};
uint8_t GY_54[] = {0x54};
uint8_t GY_51[] = {0x51};
uint8_t GY_55[] = {0x55};
uint8_t GY_Init_Command[]    = {0xA5, 0x54, 0xA5, 0x51};
uint8_t GY_Request_Command[] = {0xA5, 0x51};
uint8_t GY_Set_Command[] = {0xA5, 0x55};


void initGY(UART_HandleTypeDef *huart){
	HAL_Delay(500);
	HAL_UART_Transmit(huart, GY_A5, 1, 100);
	HAL_UART_Transmit(huart, GY_54, 1, 100);
	HAL_Delay(500);
	HAL_UART_Transmit(huart, GY_A5, 1, 100);
	HAL_UART_Transmit(huart, GY_51, 1, 100);
	HAL_Delay(500);
	HAL_UART_Transmit(huart, GY_A5, 1, 100);
	HAL_UART_Transmit(huart, GY_55, 1, 100);
	HAL_Delay(500);
}

/*
 * uart.c
 *
 *  Created on: Dec 29, 2025
 *      Author: user
 */

#include "uart.h"

UART_HandleTypeDef* pMyUartHandle;
void uart_init(UART_HandleTypeDef* pUartHandle)
{
	pMyUartHandle = pUartHandle;
}

uint16_t uart_send(uint8_t ch)
{
	return 0;
}

uint16_t uart_printf(uint8_t ch)
{
	return 0;
}

uint16_t uartAvailable(uint8_t ch)
{
	return 0;
}


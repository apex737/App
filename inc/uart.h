/*
 * uart.h
 *
 *  Created on: Dec 29, 2025
 *      Author: user
 */

#ifndef APP_INC_UART_H_
#define APP_INC_UART_H_

#include "def.h"

void uart_init(UART_HandleTypeDef* pUartHandle);
uint16_t uart_send(uint8_t ch);
uint16_t uart_printf(uint8_t ch);
uint16_t uartAvailable(uint8_t ch);

#endif /* APP_INC_UART_H_ */

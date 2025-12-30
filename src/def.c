/*
 * def.c
 *
 *  Created on: Dec 29, 2025
 *      Author: user
 */

#include "def.h"

void delay(uint16_t ms)
{
	HAL_Delay(ms);
}

uint32_t millis(void)
{
	return HAL_GetTick();
}

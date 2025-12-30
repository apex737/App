/*
 * app.h
 *
 *  Created on: Dec 29, 2025
 *      Author: user
 */

#ifndef APP_INC_APP_H_
#define APP_INC_APP_H_
#include "def.h"
#include "adc.h"
#include "uart.h"
#include "tdoa.h"

typedef struct {
	ADC_HandleTypeDef* hadc;
	UART_HandleTypeDef* huart;
	TIM_HandleTypeDef* htim_trgo;
	TIM_HandleTypeDef* htim_os;
} app_handle_t;

void app_init(app_handle_t* pApp);

void app_main(void);

#endif /* APP_INC_APP_H_ */

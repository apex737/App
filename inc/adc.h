/*
 * adc.h
 *
 *  Created on: Dec 29, 2025
 *      Author: user
 */

#ifndef APP_INC_ADC_H_
#define APP_INC_ADC_H_

#include "def.h"
#define NOC					2
#define KB					1024
#define DOUBLE_BUF_SIZE		2 * (NOC * KB)
#define SAMPLE_PER_CH		DOUBLE_BUF_SIZE / 2
#define BUFFER_OFFSET		SAMPLE_PER_CH

typedef struct {
	volatile bool half_xfer;
	volatile bool xfer_cplt;
	volatile bool awd_triggered;
	volatile bool tdoa_overrun_err;
	volatile bool tdoa_running;
} Buffer_Status_t;

// [안전장치 1] 연산 전용 버퍼를 따로 둡니다 (DMA가 건드리지 못하는 안전지대)
extern uint16_t mic1_buf[];
extern uint16_t mic2_buf[];

void adc_init(ADC_HandleTypeDef* pAdcHandle, TIM_HandleTypeDef* pTimHandle);
void split_adc_data(uint16_t offset);
void adc_awd_enable(void);


#endif /* APP_INC_ADC_H_ */

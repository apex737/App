/*
 * adc.h
 *
 *  Created on: Dec 29, 2025
 *      Author: user
 */

#ifndef APP_INC_ADC_H_
#define APP_INC_ADC_H_

#include "def.h"
#define NOC                 2
#define KB                  1024

#define SAMPLES_PER_CH      KB
#define BUF_SIZE    	    (NOC * SAMPLES_PER_CH)
#define DOUBLE_BUF_SIZE     (2 * BUF_SIZE)
#define BUFFER_OFFSET		BUF_SIZE

typedef struct {
	volatile bool half_xfer;
	volatile bool xfer_cplt;
	volatile bool awd_triggered;
	volatile bool tdoa_overrun_err;
	volatile bool tdoa_running;
} Buffer_Status_t;

// [안전장치 1] DMA가 건드리지 못하는 안전지대
extern uint16_t mic1_buf[];
extern uint16_t mic2_buf[];

void adc_init(ADC_HandleTypeDef* pAdcHandle, TIM_HandleTypeDef* pTimHandle);
void split_adc_data(uint16_t offset);
void adc_awd_enable(void);


#endif /* APP_INC_ADC_H_ */

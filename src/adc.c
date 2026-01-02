/*
 * adc.c
 *
 *  Created on: Dec 29, 2025
 *      Author: user
 */


#include "adc.h"

static uint16_t adc_dma_buf[DOUBLE_BUF_SIZE]; // adc 0 ~ 4095

// 연산 전용 버퍼
uint16_t mic1_buf[SAMPLES_PER_CH];
uint16_t mic2_buf[SAMPLES_PER_CH];
uint16_t mic3_buf[SAMPLES_PER_CH];
// 상태 플래그
Buffer_Status_t buf_status = {0};
// adc.c 내부에서만 씀
static volatile bool debouncing = false;
// 페리 핸들
ADC_HandleTypeDef* pMyAdcHandle;
TIM_HandleTypeDef* pTrgoTimHandle;
TIM_HandleTypeDef* pOsTimHandle;


void adc_init
(
	ADC_HandleTypeDef* pAdcHandle,
	TIM_HandleTypeDef* pTim1,
	TIM_HandleTypeDef* pTim2
)
{
	pMyAdcHandle = pAdcHandle;
	pTrgoTimHandle = pTim1;
	pOsTimHandle = pTim2;
	HAL_ADC_Start_DMA(pMyAdcHandle, (uint32_t *)adc_dma_buf, DOUBLE_BUF_SIZE);
	HAL_TIM_Base_Start(pTrgoTimHandle);
}

void split_adc_data(uint16_t offset)
{
	if (buf_status.tdoa_running == true)
	{
		// 메인 루프가 20ms 동안 처리를 못 끝내면 프레임 폐기
		buf_status.tdoa_overrun_err = true;
		return;
	}
	for(uint32_t j = 0; j < SAMPLES_PER_CH; j++)
	{
		uint32_t i = j * NOC; // NOC = 3
		mic1_buf[j] = adc_dma_buf[offset + i];
		mic2_buf[j] = adc_dma_buf[offset + i + 1];
		mic3_buf[j] = adc_dma_buf[offset + i + 2];
	}
	// RAW Data -> MIC 1-3 Done
	buf_status.tdoa_running = true;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == pMyAdcHandle->Instance)
	{
		buf_status.half_xfer = true; // buf0 split start
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == pMyAdcHandle->Instance)
	{
		buf_status.xfer_cplt = true; // buf1 split start
	}

}

// AWD 스레숄드 이탈 인터럽트 콜백 함수
void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance != pMyAdcHandle->Instance)
        return;

    if (debouncing)
    	return;      // 디바운스 중이면 무시

    buf_status.awd_triggered = true;

    __HAL_ADC_DISABLE_IT(hadc, ADC_IT_AWD);  // AWD 차단

    debouncing = true;

    __HAL_TIM_SET_COUNTER(pOsTimHandle, 0);
    HAL_TIM_Base_Start_IT(pOsTimHandle);   // one-shot 시작
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == pOsTimHandle->Instance)
    {
        HAL_TIM_Base_Stop_IT(pOsTimHandle);  // one-shot 중지
        debouncing = false;		// Debounce Time Elapsed
        adc_awd_enable();
    }
}


// AWD ON
void adc_awd_enable(void)
{
    // AWD 인터럽트 플래그를 먼저 지워야 안전함
    __HAL_ADC_CLEAR_FLAG(pMyAdcHandle, ADC_FLAG_AWD);
    // AWD 인터럽트 활성화
    __HAL_ADC_ENABLE_IT(pMyAdcHandle, ADC_IT_AWD);
}




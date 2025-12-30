/*
 * adc.c
 *
 *  Created on: Dec 29, 2025
 *      Author: user
 */


#include "adc.h"

// TDOA 처리를 위한 채널별 버퍼 (외부에서 참조 가능하게 헤더에 extern 선언 필요)
// 크기는 DOUBLE_BUF_SIZE / 2 (채널 수) / 2 (Ping-Pong) 가 아니라
// 단순히 (DOUBLE_BUF_SIZE / 채널수) 만큼 잡으면 됩니다.
// 예: 전체 버퍼가 2048이고 채널이 2개면 -> 각각 1024개

static uint16_t adc_dma_buf[DOUBLE_BUF_SIZE]; // adc 0 ~ 4095
// [안전장치 1] 연산 전용 버퍼를 따로 둡니다 (DMA가 건드리지 못하는 안전지대)
uint16_t mic1_buf[SAMPLES_PER_CH];
uint16_t mic2_buf[SAMPLES_PER_CH];
// [안전장치 2] 상태 플래그
Buffer_Status_t buf_status = {0};
ADC_HandleTypeDef* pMyAdcHandle;
TIM_HandleTypeDef* pTrgoTimHandle;
TIM_HandleTypeDef* pOsTimHandle;

static volatile bool debouncing = false;
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
		// 비상! 메인 루프가 20ms 동안 처리를 못 끝냈음.
		// 이번 데이터는 버립니다 (Drop) -> 데이터 꼬임 방지
		buf_status.tdoa_overrun_err = true;
		return;
	}
	for(uint32_t j = 0; j < SAMPLES_PER_CH; j++)
	{
		uint32_t i = j*2;
		mic1_buf[j] = adc_dma_buf[offset + i];
		mic2_buf[j] = adc_dma_buf[offset + i + 1];
	}
	// RAW Data -> MIC 1,2 Done
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

// [수정] 콜백 함수
void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance != pMyAdcHandle->Instance)
        return;

    if (debouncing)
        return;                     // 이미 홀드오프 중이면 무시

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
        HAL_TIM_Base_Stop_IT(pOsTimHandle);  // one-shot
        debouncing = false;
        adc_awd_enable();            // 여기서만 재무장
    }
}


// [추가] AWD를 다시 켜주는 함수
void adc_awd_enable(void)
{
    // AWD 인터럽트 플래그를 먼저 지워야 안전함
    __HAL_ADC_CLEAR_FLAG(pMyAdcHandle, ADC_FLAG_AWD);
    // AWD 인터럽트 활성화
    __HAL_ADC_ENABLE_IT(pMyAdcHandle, ADC_IT_AWD);
}




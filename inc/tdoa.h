/*
 * tdoa.h
 *
 *  Created on: Dec 29, 2025
 *      Author: user
 */

#ifndef APP_INC_TDOA_H_
#define APP_INC_TDOA_H_

#include "def.h"
#include "main.h"
#include "arm_math.h" // CMSIS-DSP 헤더 필수
#include "math.h"
// 설정값
#define FFT_SIZE      1024             // 입력 샘플 수 (반드시 2의 승수)
#define SAMPLE_RATE   50000.0f        // 샘플링 레이트 (Hz)
#define MIC_DISTANCE  0.15f           // 마이크 간격 (미터)
#define SOUND_SPEED   343.0f          // 음속 (m/s)

// 15cm 마이크 간격 최적화 세팅
#define BPF_F_LOW   150.0f   // 의미 있는 신호 확보
#define BPF_F_HIGH  950.0f   // 앨리어싱 한계(1133Hz)보다 안전하게 낮게 설정



// 구조체
typedef struct {
    arm_rfft_fast_instance_f32 fft_handler; // RFFT 핸들러
    float fft_in1[FFT_SIZE];      // Mic1 실수 입력 / IFFT 출력 공유
    float fft_in2[FFT_SIZE];      // Mic2 실수 입력
    float fft_out1[FFT_SIZE];     // Mic1 주파수 데이터 (복소수)
    float fft_out2[FFT_SIZE];     // Mic2 주파수 데이터 (복소수)
    float phat_out[FFT_SIZE];     // PHAT 연산 결과 (복소수)
} TDOA_Context_t;

// 함수 선언
void tdoa_init(void);
float tdoa_process(uint16_t* mic1, uint16_t* mic2);

#endif /* APP_INC_TDOA_H_ */

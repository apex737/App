/*
 * tdoa.c
 *
 *  Created on: Dec 29, 2025
 *      Author: user
 */

#include "tdoa.h"

static TDOA_Context_t tdoa_ctx;

// 초기화: FFT 인스턴스 설정
void tdoa_init(void)
{
    arm_rfft_fast_init_f32(&tdoa_ctx.fft_handler, FFT_SIZE);
}

// (옵션) Hann window를 쓰고 싶으면 여기에 테이블을 두거나 런타임 생성
// static float hann[FFT_SIZE];

static void remove_dc_offset(uint16_t* src, float* dst, uint16_t len)
{
    float sum = 0.0f;
    for (uint32_t i = 0; i < len; i++) sum += (float)src[i];
    float mean = sum / (float)len;

    for (uint32_t i = 0; i < len; i++)
    {
        dst[i] = (float)src[i] - mean;
        // dst[i] *= hann[i]; // leakage 줄이고 싶으면 enable
    }
}

static void gcc_phat_rfft_packed(const float* X1, const float* X2, float* RES, uint32_t N)
{
    // N = FFT_SIZE (power of 2)
    // arm_rfft_fast_f32 packed format:
    // out[0] = Re{X[0]}  (DC)
    // out[1] = Re{X[N/2]}(Nyquist)
    // out[2k]   = Re{X[k]}, out[2k+1] = Im{X[k]} for k=1..N/2-1

    // DC bin (k=0): purely real
    {
        float a = X1[0];
        float b = X2[0];
        float r = a * b;               // conj is same (real)
        float mag = fabsf(r);
        RES[0] = (mag > 1e-6f) ? (r / mag) : 0.0f;   // +1 or -1
    }

    // Nyquist bin (k=N/2): stored at index 1, purely real
    {
        float a = X1[1];
        float b = X2[1];
        float r = a * b;
        float mag = fabsf(r);
        RES[1] = (mag > 1e-6f) ? (r / mag) : 0.0f;   // +1 or -1
    }

    // k = 1 .. N/2 - 1
    for (uint32_t k = 1; k < (N / 2); k++)
    {
        uint32_t i = 2U * k;

        float re1 = X1[i];
        float im1 = X1[i + 1U];

        float re2 = X2[i];
        float im2 = X2[i + 1U];

        // X1 * conj(X2)
        float res_re = (re1 * re2) + (im1 * im2);
        float res_im = (im1 * re2) - (re1 * im2);

        float mag = sqrtf(res_re * res_re + res_im * res_im);
        if (mag > 1e-6f)
        {
            RES[i]       = res_re / mag;
            RES[i + 1U]  = res_im / mag;
        }
        else
        {
            RES[i]       = 0.0f;
            RES[i + 1U]  = 0.0f;
        }
    }
}

static int find_peak_lag_limited(const float* corr, uint32_t N, int max_lag, float* out_peak_val)
{
    // corr: length N, circular correlation (ifft output)
    // valid lag range: [-max_lag, +max_lag]
    // mapping:
    //   lag >= 0 => index = lag
    //   lag < 0  => index = N + lag

    float best = -1e30f;
    int best_lag = 0;

    // +lag: 0..max_lag
    for (int lag = 0; lag <= max_lag; lag++)
    {
        float v = corr[lag];
        if (v > best)
        {
            best = v;
            best_lag = lag;
        }
    }

    // -lag: N-max_lag .. N-1
    for (int idx = (int)N - max_lag; idx < (int)N; idx++)
    {
        float v = corr[idx];
        int lag = idx - (int)N; // negative
        if (v > best)
        {
            best = v;
            best_lag = lag;
        }
    }

    if (out_peak_val) *out_peak_val = best;
    return best_lag;
}

float tdoa_process(uint16_t* mic1, uint16_t* mic2)
{
    // 0) (방어) init 안 됐으면 여기서라도 초기화 (원래는 app_init에서 1회만)
    // 이 방어를 싫어하면 제거하고, 반드시 app_init에서 tdoa_init 호출
    static int inited = 0;
    if (!inited) { tdoa_init(); inited = 1; }

    // 1) 전처리: DC 제거 + float 변환
    remove_dc_offset(mic1, tdoa_ctx.fft_in1, FFT_SIZE);
    remove_dc_offset(mic2, tdoa_ctx.fft_in2, FFT_SIZE);

    // 2) FFT
    arm_rfft_fast_f32(&tdoa_ctx.fft_handler, tdoa_ctx.fft_in1, tdoa_ctx.fft_out1, 0);
    arm_rfft_fast_f32(&tdoa_ctx.fft_handler, tdoa_ctx.fft_in2, tdoa_ctx.fft_out2, 0);

    // 3) GCC-PHAT (packed 포맷 기반)
    gcc_phat_rfft_packed(tdoa_ctx.fft_out1, tdoa_ctx.fft_out2, tdoa_ctx.phat_out, FFT_SIZE);

    // 4) IFFT => corr (길이 N)
    arm_rfft_fast_f32(&tdoa_ctx.fft_handler, tdoa_ctx.phat_out, tdoa_ctx.fft_in1, 1);

    // 5) 물리적으로 가능한 lag 범위에서만 peak 탐색
    int max_lag = (int)roundf((MIC_DISTANCE / SOUND_SPEED) * SAMPLE_RATE);
    if (max_lag < 1) max_lag = 1;
    if (max_lag > (int)(FFT_SIZE / 2 - 1)) max_lag = (int)(FFT_SIZE / 2 - 1);

    float peak_val = 0.0f;
    int shift_lag = find_peak_lag_limited(tdoa_ctx.fft_in1, FFT_SIZE, max_lag, &peak_val);

    // 6) lag -> time -> distance diff
    float time_delay = (float)shift_lag / SAMPLE_RATE;
    float dist_diff  = time_delay * SOUND_SPEED;

    // 7) asin argument clamp
    float argument = dist_diff / MIC_DISTANCE;
    if (argument > 1.0f)  argument = 1.0f;
    if (argument < -1.0f) argument = -1.0f;

    float angle_rad = asinf(argument);
    float angle_deg = angle_rad * (180.0f / 3.1415926f);

    // (옵션) 신뢰도 낮으면 이전 값 유지 같은 로직을 여기서 추가 가능
    // 예: peak_val이 너무 낮으면 return last_angle;

    return angle_deg;
}

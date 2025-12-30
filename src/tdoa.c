/*
 * tdoa.c
 *
 *  Created on: Dec 29, 2025
 *      Author: user
 */

#include "tdoa.h"

static TDOA_Context_t tdoa_ctx;

void tdoa_init(void)
{
    arm_rfft_fast_init_f32(&tdoa_ctx.fft_handler, FFT_SIZE);
}

static void remove_dc_offset(uint16_t* src, float* dst, uint16_t len)
{
    float sum = 0.0f;
    for (uint32_t i = 0; i < len; i++) sum += (float)src[i];
    float mean = sum / (float)len;

    for (uint32_t i = 0; i < len; i++)
    {
        dst[i] = (float)src[i] - mean;
    }
}

/*
 * 주파수 도메인 BPF (bin 마스킹)
 * - arm_rfft_fast_f32 packed format 기반으로 직접 bin을 0 처리
 * - keep band: [f_low, f_high]
 */
static void apply_bpf_rfft_packed(float* X, uint32_t N, float fs, float f_low, float f_high)
{
    // packed format:
    // X[0] = Re{X0} (DC)
    // X[1] = Re{X_{N/2}} (Nyquist)
    // for k=1..N/2-1: X[2k]=Re{Xk}, X[2k+1]=Im{Xk}

    // DC 제거: 항상 0으로 (이미 time-domain에서 mean 제거했지만 안전하게)
    X[0] = 0.0f;

    // Nyquist (fs/2)는 보통 불필요, 대역 밖이면 0 처리
    {
        float f_nyq = fs * 0.5f;
        if (f_nyq < f_low || f_nyq > f_high) X[1] = 0.0f;
    }

    // 일반 bin
    for (uint32_t k = 1; k < (N / 2); k++)
    {
        float f = ((float)k * fs) / (float)N;
        if (f < f_low || f > f_high)
        {
            uint32_t i = 2U * k;
            X[i]     = 0.0f; // Re
            X[i + 1] = 0.0f; // Im
        }
    }
}

static void gcc_phat_rfft_packed(const float* X1, const float* X2, float* RES, uint32_t N)
{
    // DC bin (k=0): purely real
    {
        float a = X1[0];
        float b = X2[0];
        float r = a * b;
        float mag = fabsf(r);
        RES[0] = (mag > 1e-6f) ? (r / mag) : 0.0f;
    }

    // Nyquist bin (k=N/2): stored at index 1, purely real
    {
        float a = X1[1];
        float b = X2[1];
        float r = a * b;
        float mag = fabsf(r);
        RES[1] = (mag > 1e-6f) ? (r / mag) : 0.0f;
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
            RES[i]      = res_re / mag;
            RES[i + 1U] = res_im / mag;
        }
        else
        {
            RES[i]      = 0.0f;
            RES[i + 1U] = 0.0f;
        }
    }
}

static int find_peak_lag_limited(const float* corr, uint32_t N, int max_lag, float* out_peak_val)
{
    float best = -1e30f;
    int best_lag = 0;

    // +lag
    for (int lag = 0; lag <= max_lag; lag++)
    {
        float v = corr[lag];
        if (v > best)
        {
            best = v;
            best_lag = lag;
        }
    }

    // -lag (wrap)
    for (int idx = (int)N - max_lag; idx < (int)N; idx++)
    {
        float v = corr[idx];
        int lag = idx - (int)N;
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

    // 1) DC 제거 + float 변환
    remove_dc_offset(mic1, tdoa_ctx.fft_in1, FFT_SIZE);
    remove_dc_offset(mic2, tdoa_ctx.fft_in2, FFT_SIZE);

    // 2) FFT
    arm_rfft_fast_f32(&tdoa_ctx.fft_handler, tdoa_ctx.fft_in1, tdoa_ctx.fft_out1, 0);
    arm_rfft_fast_f32(&tdoa_ctx.fft_handler, tdoa_ctx.fft_in2, tdoa_ctx.fft_out2, 0);

    // 2.5) BPF 적용 (주파수 도메인 마스킹)
    apply_bpf_rfft_packed(tdoa_ctx.fft_out1, FFT_SIZE, SAMPLE_RATE, BPF_F_LOW, BPF_F_HIGH);
    apply_bpf_rfft_packed(tdoa_ctx.fft_out2, FFT_SIZE, SAMPLE_RATE, BPF_F_LOW, BPF_F_HIGH);

    // 3) GCC-PHAT
    gcc_phat_rfft_packed(tdoa_ctx.fft_out1, tdoa_ctx.fft_out2, tdoa_ctx.phat_out, FFT_SIZE);

    // 4) IFFT => corr
    arm_rfft_fast_f32(&tdoa_ctx.fft_handler, tdoa_ctx.phat_out, tdoa_ctx.fft_in1, 1);

    // 5) 물리 lag 범위 제한 peak 탐색
    int max_lag = (int)roundf((MIC_DISTANCE / SOUND_SPEED) * SAMPLE_RATE);
    if (max_lag < 1) max_lag = 1;
    if (max_lag > (int)(FFT_SIZE / 2 - 1)) max_lag = (int)(FFT_SIZE / 2 - 1);

    float peak_val = 0.0f;
    int shift_lag = find_peak_lag_limited(tdoa_ctx.fft_in1, FFT_SIZE, max_lag, &peak_val);

    // 6) lag -> angle
    float time_delay = (float)shift_lag / SAMPLE_RATE;
    float dist_diff  = time_delay * SOUND_SPEED;

    float argument = dist_diff / MIC_DISTANCE;
    if (argument > 1.0f)  argument = 1.0f;
    if (argument < -1.0f) argument = -1.0f;

    float angle_rad = asinf(argument);
    float angle_deg = angle_rad * (180.0f / 3.1415926f);

    return angle_deg;
}

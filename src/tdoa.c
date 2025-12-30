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

/* ================= BPF (freq-domain) ================= */

static void apply_bpf_rfft_packed(float* X, uint32_t N, float fs,
                                  float f_low, float f_high)
{
    X[0] = 0.0f; // DC

    float f_nyq = fs * 0.5f;
    if (f_nyq < f_low || f_nyq > f_high)
        X[1] = 0.0f;

    for (uint32_t k = 1; k < (N / 2); k++)
    {
        float f = ((float)k * fs) / (float)N;
        if (f < f_low || f > f_high)
        {
            uint32_t i = 2U * k;
            X[i]     = 0.0f;
            X[i + 1] = 0.0f;
        }
    }
}

/* ================= GCC-PHAT ================= */

static void gcc_phat_rfft_packed(const float* X1,
                                 const float* X2,
                                 float* RES,
                                 uint32_t N)
{
    // DC
    {
        float r = X1[0] * X2[0];
        float mag = fabsf(r);
        RES[0] = (mag > 1e-6f) ? (r / mag) : 0.0f;
    }

    // Nyquist
    {
        float r = X1[1] * X2[1];
        float mag = fabsf(r);
        RES[1] = (mag > 1e-6f) ? (r / mag) : 0.0f;
    }

    for (uint32_t k = 1; k < (N / 2); k++)
    {
        uint32_t i = 2U * k;

        float re1 = X1[i];
        float im1 = X1[i + 1];
        float re2 = X2[i];
        float im2 = X2[i + 1];

        float res_re = (re1 * re2) + (im1 * im2);
        float res_im = (im1 * re2) - (re1 * im2);

        float mag = sqrtf(res_re * res_re + res_im * res_im);
        if (mag > 1e-6f)
        {
            RES[i]     = res_re / mag;
            RES[i + 1] = res_im / mag;
        }
        else
        {
            RES[i]     = 0.0f;
            RES[i + 1] = 0.0f;
        }
    }
}

/* ================= Peak Search ================= */

static int find_peak_lag_limited(const float* corr,
                                 uint32_t N,
                                 int max_lag,
                                 float* out_peak_val)
{
    float best = -1e30f;
    int best_lag = 0;

    for (int lag = 0; lag <= max_lag; lag++)
    {
        float v = corr[lag];
        if (v > best)
        {
            best = v;
            best_lag = lag;
        }
    }

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

/* ================= Main ================= */

float tdoa_process(uint16_t* mic1, uint16_t* mic2)
{
    // 1) DC 제거
    remove_dc_offset(mic1, tdoa_ctx.fft_in1, FFT_SIZE);
    remove_dc_offset(mic2, tdoa_ctx.fft_in2, FFT_SIZE);

    // 2) FFT
    arm_rfft_fast_f32(&tdoa_ctx.fft_handler,
                      tdoa_ctx.fft_in1,
                      tdoa_ctx.fft_out1, 0);
    arm_rfft_fast_f32(&tdoa_ctx.fft_handler,
                      tdoa_ctx.fft_in2,
                      tdoa_ctx.fft_out2, 0);

    // 3) BPF
    apply_bpf_rfft_packed(tdoa_ctx.fft_out1, FFT_SIZE,
                          SAMPLE_RATE, BPF_F_LOW, BPF_F_HIGH);
    apply_bpf_rfft_packed(tdoa_ctx.fft_out2, FFT_SIZE,
                          SAMPLE_RATE, BPF_F_LOW, BPF_F_HIGH);

    // 4) GCC-PHAT
    gcc_phat_rfft_packed(tdoa_ctx.fft_out1,
                          tdoa_ctx.fft_out2,
                          tdoa_ctx.phat_out,
                          FFT_SIZE);

    // 5) IFFT → correlation
    arm_rfft_fast_f32(&tdoa_ctx.fft_handler,
                      tdoa_ctx.phat_out,
                      tdoa_ctx.fft_in1, 1);

    // 6) peak search (physics-limited)
    int max_lag = (int)roundf((MIC_DISTANCE / SOUND_SPEED) * SAMPLE_RATE);
    if (max_lag < 1) max_lag = 1;
    if (max_lag > (int)(FFT_SIZE / 2 - 1))
        max_lag = (int)(FFT_SIZE / 2 - 1);

    float peak_val;
    int k = find_peak_lag_limited(tdoa_ctx.fft_in1,
                                  FFT_SIZE, max_lag, &peak_val);

    /* ---------- Parabolic interpolation ---------- */
    float lag_f = (float)k;

    int k_prev = (k - 1 + FFT_SIZE) % FFT_SIZE;
    int k_next = (k + 1) % FFT_SIZE;

    float y_prev = tdoa_ctx.fft_in1[k_prev];
    float y0     = tdoa_ctx.fft_in1[k];
    float y_next = tdoa_ctx.fft_in1[k_next];

    float denom = (y_prev - 2.0f * y0 + y_next);
    if (fabsf(denom) > 1e-6f)
    {
        float delta = 0.5f * (y_prev - y_next) / denom;
        if (delta > 0.5f)  delta = 0.5f;
        if (delta < -0.5f) delta = -0.5f;
        lag_f += delta;
    }
    /* --------------------------------------------- */

    // 7) lag → angle
    float time_delay = lag_f / SAMPLE_RATE;
    float dist_diff  = time_delay * SOUND_SPEED;

    float arg = dist_diff / MIC_DISTANCE;
    if (arg > 1.0f)  arg = 1.0f;
    if (arg < -1.0f) arg = -1.0f;

    return asinf(arg) * (180.0f / 3.1415926f);
}

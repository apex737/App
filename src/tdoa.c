/* =========================================================
 * tdoa.c  (3-MIC, 정삼각형 배치)
 * ========================================================= */

#include "tdoa.h"

static TDOA_Context_t tdoa_ctx;

static float prev_arg;

/* =========================
 *  Init
 * ========================= */
void tdoa_init(void)
{
    arm_rfft_fast_init_f32(&tdoa_ctx.fft_handler, FFT_SIZE);
    prev_arg = 0.0f;
}

/* =========================
 *  Preprocessing
 * ========================= */
static void remove_dc_offset(uint16_t* src, float* dst, uint16_t len)
{
    float sum = 0.0f;
    for (uint32_t i = 0; i < len; i++)
        sum += (float)src[i];

    float mean = sum / (float)len;

    for (uint32_t i = 0; i < len; i++)
        dst[i] = (float)src[i] - mean;
}

/* =========================
 *  Frequency-domain BPF
 * ========================= */
static void apply_bpf_rfft_packed(float* X, uint32_t N,
                                  float fs, float f_low, float f_high)
{
    /* DC */
    X[0] = 0.0f;

    /* Nyquist */
    float f_nyq = fs * 0.5f;
    if (f_nyq < f_low || f_nyq > f_high)
        X[1] = 0.0f;

    /* Normal bins */
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

/* =========================
 *  GCC-PHAT (packed RFFT)
 * ========================= */
static void gcc_phat_rfft_packed(const float* X1,
                                 const float* X2,
                                 float* RES,
                                 uint32_t N)
{
    /* DC */
    {
        float r = X1[0] * X2[0];
        float mag = fabsf(r);
        RES[0] = (mag > 1e-6f) ? (r / mag) : 0.0f;
    }

    /* Nyquist */
    {
        float r = X1[1] * X2[1];
        float mag = fabsf(r);
        RES[1] = (mag > 1e-6f) ? (r / mag) : 0.0f;
    }

    /* Normal bins */
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


// 안전한 배열 접근을 위한 헬퍼
static float get_val_safe(const float* arr, int idx, uint32_t N)
{
    if (idx < 0) idx += N;
    else if (idx >= (int)N) idx -= N;
    return arr[idx];
}

/* =========================
 *  Peak utilities
 * ========================= */
static float find_peak_lag_subsample(const float* corr, uint32_t N, float* out_peak_val)
{
    float best_val = -1e30f;
    int best_idx = 0;
    int best_lag_int = 0;

    // +lag
    for (int lag = 0; lag <= MAX_LAG; lag++) {
        float v = corr[lag];
        if (v > best_val) { best_val = v; best_idx = lag; best_lag_int = lag; }
    }
    // -lag
    for (int i = (int)N - MAX_LAG; i < (int)N; i++) {
        float v = corr[i];
        if (v > best_val) { best_val = v; best_idx = i; best_lag_int = i - (int)N; }
    }

    if (out_peak_val) *out_peak_val = best_val;

    // 2차 보간
    float y_curr  = best_val;
    float y_left  = get_val_safe(corr, best_idx - 1, N);
    float y_right = get_val_safe(corr, best_idx + 1, N);

    float denominator = 2.0f * (y_left - 2.0f * y_curr + y_right);
    float delta = 0.0f;

    if (fabsf(denominator) > 1e-9f) delta = (y_left - y_right) / denominator;
    if (delta > 0.5f) delta = 0.5f;
    if (delta < -0.5f) delta = -0.5f;

    return (float)best_lag_int + delta;
}

static float tdoa_pair_lag(uint16_t* micA, uint16_t* micB)
{
    // 1) DC 제거
    remove_dc_offset(micA, tdoa_ctx.fft_in1, FFT_SIZE);
    remove_dc_offset(micB, tdoa_ctx.fft_in2, FFT_SIZE);

    // 2) FFT
    arm_rfft_fast_f32(&tdoa_ctx.fft_handler, tdoa_ctx.fft_in1, tdoa_ctx.fft_out1, 0);
    arm_rfft_fast_f32(&tdoa_ctx.fft_handler, tdoa_ctx.fft_in2, tdoa_ctx.fft_out2, 0);

    // 3) BPF
    apply_bpf_rfft_packed(tdoa_ctx.fft_out1, FFT_SIZE, SAMPLE_RATE, BPF_F_LOW, BPF_F_HIGH);
    apply_bpf_rfft_packed(tdoa_ctx.fft_out2, FFT_SIZE, SAMPLE_RATE, BPF_F_LOW, BPF_F_HIGH);

    // 4) GCC-PHAT
    gcc_phat_rfft_packed(tdoa_ctx.fft_out1, tdoa_ctx.fft_out2, tdoa_ctx.phat_out, FFT_SIZE);

    // 5) IFFT
    arm_rfft_fast_f32(&tdoa_ctx.fft_handler, tdoa_ctx.phat_out, tdoa_ctx.fft_in1, 1);

    // 7) Sub-sample Peak Lag
    return find_peak_lag_subsample(tdoa_ctx.fft_in1, FFT_SIZE, NULL);
}

/* =========================
 * 3-MIC Process (Least Squares Estimation 적용)
 * ========================= */
// Far-Field 근사, 정삼각형 배치 최적화 수식
float tdoa_process_3mic
(uint16_t* mic1, uint16_t* mic2, uint16_t* mic3,
        float* lagExp, float* err)
{
    // 1. 3개의 쌍(Pair)에 대해 Lag 계산
    float lag12 = tdoa_pair_lag(mic1, mic2);
    float lag13 = tdoa_pair_lag(mic1, mic3);
    float lag23 = tdoa_pair_lag(mic2, mic3);

    // 2. 신뢰성 검증
    *lagExp = lag13 - lag12;
    *err = fabsf(lag23 - *lagExp);

    // 3. 거리 변환
    float d12 = (lag12 / SAMPLE_RATE) * SOUND_SPEED;
    float d13 = (lag13 / SAMPLE_RATE) * SOUND_SPEED;
    float d23 = (lag23 / SAMPLE_RATE) * SOUND_SPEED;

    // 4. LSE 정규 방정식; 3개의 데이터를 모두 사용하여 X, Y 성분을 추정
    // 정삼각형에서 각 변에 투영된 성분을 평균
    float ux = -(2.0f * d12 + d13 - d23) / (3.0f * MIC_DISTANCE);
    float uy = -(d13 + d23) / (1.7320508f * MIC_DISTANCE); // 1.732... = sqrt(3)

    // 5.각도 산출
    float ang = atan2f(uy, ux) * (180.0f / 3.1415926f);
    if (ang < 0.0f) ang += 360.0f;

    return ang;
}

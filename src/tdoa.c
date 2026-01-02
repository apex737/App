/* =========================================================
 * tdoa.c  (3-MIC, 정삼각형 배치)
 * - 기존 2-MIC tdoa_process() 원형 유지
 * - 내부 코어를 재사용해 pairwise lag 함수 추가
 * - 3-MIC 래퍼에서 0~360° 산출
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

/* =========================
 *  Peak utilities
 * ========================= */
static int find_peak_lag_limited(const float* corr,
                                 uint32_t N,
                                 int max_lag,
                                 float* out_peak)
{
    float best = -1e30f;
    int best_lag = 0;

    /* +lag */
    for (int lag = 0; lag <= max_lag; lag++)
    {
        float v = corr[lag];
        if (v > best)
        {
            best = v;
            best_lag = lag;
        }
    }

    /* -lag */
    for (int i = (int)N - max_lag; i < (int)N; i++)
    {
        float v = corr[i];
        int lag = i - (int)N;
        if (v > best)
        {
            best = v;
            best_lag = lag;
        }
    }

    if (out_peak) *out_peak = best;
    return best_lag;
}


/* =========================
 *  Main TDOA
 * ========================= */
float tdoa_process(uint16_t* mic1, uint16_t* mic2)
{
    /* 1) DC 제거 */
    remove_dc_offset(mic1, tdoa_ctx.fft_in1, FFT_SIZE);
    remove_dc_offset(mic2, tdoa_ctx.fft_in2, FFT_SIZE);

    /* 2) FFT */
    arm_rfft_fast_f32(&tdoa_ctx.fft_handler,
                      tdoa_ctx.fft_in1,
                      tdoa_ctx.fft_out1, 0);

    arm_rfft_fast_f32(&tdoa_ctx.fft_handler,
                      tdoa_ctx.fft_in2,
                      tdoa_ctx.fft_out2, 0);

    /* 3) BPF */
    apply_bpf_rfft_packed(tdoa_ctx.fft_out1,
                          FFT_SIZE,
                          SAMPLE_RATE,
                          BPF_F_LOW,
                          BPF_F_HIGH);

    apply_bpf_rfft_packed(tdoa_ctx.fft_out2,
                          FFT_SIZE,
                          SAMPLE_RATE,
                          BPF_F_LOW,
                          BPF_F_HIGH);

    /* 4) GCC-PHAT */
    gcc_phat_rfft_packed(tdoa_ctx.fft_out1,
                         tdoa_ctx.fft_out2,
                         tdoa_ctx.phat_out,
                         FFT_SIZE);

    /* 5) IFFT → correlation */
    arm_rfft_fast_f32(&tdoa_ctx.fft_handler,
                      tdoa_ctx.phat_out,
                      tdoa_ctx.fft_in1, 1);

    /* 6) 물리적 lag 제한 */
    int max_lag = (int)roundf((MIC_DISTANCE / SOUND_SPEED) * SAMPLE_RATE);
    if (max_lag < 1) max_lag = 1;
    if (max_lag > (int)(FFT_SIZE / 2 - 1))
        max_lag = (int)(FFT_SIZE / 2 - 1);

    /* 7) Peak Lag 탐색 */
	float peak_val = 0.0f;
	int raw_lag = find_peak_lag_limited(tdoa_ctx.fft_in1, FFT_SIZE, max_lag, &peak_val);

	/* 8) Lag → Angle 변환 */
	float time_delay = (float)raw_lag / SAMPLE_RATE;
	float dist_diff  = time_delay * SOUND_SPEED;

	// 아크사인 도메인(-1 ~ 1) 체크
	float argument = dist_diff / MIC_DISTANCE;
	if (argument > 1.0f || argument < -1.0f)  argument = prev_arg;
	prev_arg = argument;

	return asinf(argument) * (180.0f / 3.1415926f);
}


/* =========================
 *  Pairwise lag core (추가)
 *  - 2MIC tdoa_process()의 1~7단계를 그대로 재사용
 *  - 결과로 raw_lag 반환
 * ========================= */
static int tdoa_pair_lag(uint16_t* micA, uint16_t* micB, float* out_peak)
{
    /* 1) DC 제거 */
    remove_dc_offset(micA, tdoa_ctx.fft_in1, FFT_SIZE);
    remove_dc_offset(micB, tdoa_ctx.fft_in2, FFT_SIZE);

    /* 2) FFT */
    arm_rfft_fast_f32(&tdoa_ctx.fft_handler,
                      tdoa_ctx.fft_in1,
                      tdoa_ctx.fft_out1, 0);

    arm_rfft_fast_f32(&tdoa_ctx.fft_handler,
                      tdoa_ctx.fft_in2,
                      tdoa_ctx.fft_out2, 0);

    /* 3) BPF */
    apply_bpf_rfft_packed(tdoa_ctx.fft_out1,
                          FFT_SIZE,
                          SAMPLE_RATE,
                          BPF_F_LOW,
                          BPF_F_HIGH);

    apply_bpf_rfft_packed(tdoa_ctx.fft_out2,
                          FFT_SIZE,
                          SAMPLE_RATE,
                          BPF_F_LOW,
                          BPF_F_HIGH);

    /* 4) GCC-PHAT */
    gcc_phat_rfft_packed(tdoa_ctx.fft_out1,
                         tdoa_ctx.fft_out2,
                         tdoa_ctx.phat_out,
                         FFT_SIZE);

    /* 5) IFFT → correlation */
    arm_rfft_fast_f32(&tdoa_ctx.fft_handler,
                      tdoa_ctx.phat_out,
                      tdoa_ctx.fft_in1, 1);

    /* 6) 물리적 lag 제한 (각 pair baseline을 MIC_DISTANCE로 가정) */
    int max_lag = (int)roundf((MIC_DISTANCE / SOUND_SPEED) * SAMPLE_RATE);
    if (max_lag < 1) max_lag = 1;
    if (max_lag > (int)(FFT_SIZE / 2 - 1))
        max_lag = (int)(FFT_SIZE / 2 - 1);

    /* 7) Peak Lag 탐색 */
    float peak_val = 0.0f;
    int raw_lag = find_peak_lag_limited(tdoa_ctx.fft_in1, FFT_SIZE, max_lag, &peak_val);
    if (out_peak) *out_peak = peak_val;
    return raw_lag;
}

/* =========================
 *  3-MIC (equilateral) → 0~360 deg
 *
 *  최소 변경 / 기본형:
 *  - τ12, τ13만 사용 (M1 기준)
 *  - 일관성 체크는 가볍게만(peak 기반 optional)
 * ========================= */
/* =========================
 * 3-MIC (equilateral) → 0~360 deg
 * 개선된 로직: Far-Field 기하학 모델 사용
 * ========================= */
float tdoa_process_3mic(uint16_t* mic1, uint16_t* mic2, uint16_t* mic3)
{
    /* 1) pairwise lag 계산 */
    float p12 = 0.0f, p13 = 0.0f;
    int lag12 = tdoa_pair_lag(mic1, mic2, &p12); // tdoa_ctx 재사용 문제 없음 (순차 실행)
    int lag13 = tdoa_pair_lag(mic1, mic3, &p13);

    /* 2) lag → 거리차(미터) */
    // d12 = r2 - r1 (mic1 기준, mic2가 더 멀면 양수)
    float d12 = ((float)lag12 / SAMPLE_RATE) * SOUND_SPEED;
    float d13 = ((float)lag13 / SAMPLE_RATE) * SOUND_SPEED;

    /* 3) 물리적 한계 클램프 (안전장치) */
    if (d12 >  MIC_DISTANCE) d12 =  MIC_DISTANCE;
    if (d12 < -MIC_DISTANCE) d12 = -MIC_DISTANCE;
    if (d13 >  MIC_DISTANCE) d13 =  MIC_DISTANCE;
    if (d13 < -MIC_DISTANCE) d13 = -MIC_DISTANCE;

    /* 4) 각도 산출 (Far-Field Model)
     * 정삼각형 배치 (Mic1:원점, Mic2:X축 위)에서:
     * cos(theta) = -d12 / D
     * sin(theta) = (d12 - 2*d13) / (D * sqrt(3))
     *
     * atan2를 쓸 때는 분모 D가 약분되므로 아래와 같이 간소화 가능
     */
    float x_vec = -d12;
    float y_vec = (d12 - 2.0f * d13) / 1.7320508f; // sqrt(3)

    /* 5) 0~360 deg 변환 */
    float ang = atan2f(y_vec, x_vec) * (180.0f / 3.1415926f);

    // atan2는 -180 ~ +180을 반환하므로 0~360으로 보정
    if (ang < 0.0f) ang += 360.0f;

    return ang;
}

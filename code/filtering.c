#include "filtering.h"


#pragma section all "cpu1_dsram"

//------------------------------------------------------------------------------------------------------------------
// 一维卡尔曼滤波器：初始化
//------------------------------------------------------------------------------------------------------------------
void kalman_init(KalmanFilter *kf, float Q, float R, float initial_value)
{
    kf->x = initial_value;
    kf->P = 1.0f;
    kf->Q = Q;
    kf->R = R;
    kf->K = 0.0f;
}

//------------------------------------------------------------------------------------------------------------------
// 一维卡尔曼滤波器：输入测量值，返回滤波后的估计值
//------------------------------------------------------------------------------------------------------------------
float kalman_update(KalmanFilter *kf, float measurement)
{
    // 预测
    kf->P = kf->P + kf->Q;

    // 更新
    kf->K = kf->P / (kf->P + kf->R);
    kf->x = kf->x + kf->K * (measurement - kf->x);
    kf->P = (1.0f - kf->K) * kf->P;

    return kf->x;
}

// ===================== 3 点中值滤波器 =====================
void median3_init(MedianFilter3 *mf)
{
    mf->buf[0] = 0;
    mf->buf[1] = 0;
    mf->buf[2] = 0;
    mf->count  = 0;
}

// 返回 3 个值的中值（内部排序，不修改原数组）
static int32 median_of_three(int32 a, int32 b, int32 c)
{
    if (a > b) { int32 t = a; a = b; b = t; }
    if (b > c) { int32 t = b; b = c; c = t; }
    if (a > b) { b = a; }
    return b;
}

int32 median3_update(MedianFilter3 *mf, int32 input)
{
    // 移位：oldest ← last ← newest ← input
    mf->buf[2] = mf->buf[1];
    mf->buf[1] = mf->buf[0];
    mf->buf[0] = input;

    if (mf->count < 3) mf->count++;
    if (mf->count < 3) return input;  // 不足 3 点时直接输出

    return median_of_three(mf->buf[0], mf->buf[1], mf->buf[2]);
}

// ===================== 一阶低通滤波器 =====================
void lowpass_init(LowPassFilter *lp, float alpha)
{
    lp->alpha  = alpha;
    lp->last   = 0.0f;
    lp->inited = 0;
}

int32 lowpass_update(LowPassFilter *lp, int32 input)
{
    if (!lp->inited)
    {
        lp->last   = (float)input;
        lp->inited = 1;
        return input;
    }
    lp->last = lp->alpha * (float)input + (1.0f - lp->alpha) * lp->last;
    return (int32)lp->last;
}

// ===================== 变化率限幅器 =====================
void slew_init(SlewLimiter *sl, float max_delta)
{
    sl->max_delta = max_delta;
    sl->last      = 0.0f;
    sl->inited    = 0;
}

int32 slew_update(SlewLimiter *sl, int32 input)
{
    if (!sl->inited)
    {
        sl->last   = (float)input;
        sl->inited = 1;
        return input;
    }
    float diff = (float)input - sl->last;
    if (diff >  sl->max_delta) diff =  sl->max_delta;
    if (diff < -sl->max_delta) diff = -sl->max_delta;
    sl->last += diff;
    return (int32)sl->last;
}

// ===================== 组合滤波器 =====================
void composite_filter_init(CompositeFilter *cf, float alpha, float max_delta)
{
    median3_init(&cf->median);
    slew_init(&cf->slew, max_delta);
    lowpass_init(&cf->lowpass, alpha);
}

int32 composite_filter_update(CompositeFilter *cf, int32 input)
{
    int32 step1 = median3_update(&cf->median, input);   // 去尖峰
    int32 step2 = slew_update(&cf->slew, step1);        // 限幅
    int32 step3 = lowpass_update(&cf->lowpass, step2);   // 平滑
    return step3;
}

// ===================== IMU 陀螺仪三轴卡尔曼实例 =====================
KalmanFilter kf_gyro_x;
KalmanFilter kf_gyro_y;
KalmanFilter kf_gyro_z;

// ===================== IMU 加速度计三轴卡尔曼实例 =====================
KalmanFilter kf_acc_x;
KalmanFilter kf_acc_y;
KalmanFilter kf_acc_z;

#pragma section all restore

// 陀螺仪三轴滤波器初始化
void gyro_kalman_init(float Q, float R)
{
    kalman_init(&kf_gyro_x, Q, R, 0.0f);
    kalman_init(&kf_gyro_y, Q, R, 0.0f);
    kalman_init(&kf_gyro_z, Q, R, 0.0f);
}

// 陀螺仪三轴滤波更新
void gyro_kalman_update_all(float *gyro_x, float *gyro_y, float *gyro_z)
{
    *gyro_x = kalman_update(&kf_gyro_x, *gyro_x);
    *gyro_y = kalman_update(&kf_gyro_y, *gyro_y);
    *gyro_z = kalman_update(&kf_gyro_z, *gyro_z);
}

// 加速度计三轴滤波器初始化
void acc_kalman_init(float Q, float R)
{
    kalman_init(&kf_acc_x, Q, R, 0.0f);
    kalman_init(&kf_acc_y, Q, R, 0.0f);
    kalman_init(&kf_acc_z, Q, R, 0.0f);
}

// 加速度计三轴滤波更新
void acc_kalman_update_all(float *acc_x, float *acc_y, float *acc_z)
{
    *acc_x = kalman_update(&kf_acc_x, *acc_x);
    *acc_y = kalman_update(&kf_acc_y, *acc_y);
    *acc_z = kalman_update(&kf_acc_z, *acc_z);
}

// 一键初始化所有 IMU 滤波器
void imu_kalman_init_all(float gyro_Q, float gyro_R, float acc_Q, float acc_R)
{
    gyro_kalman_init(gyro_Q, gyro_R);
    acc_kalman_init(acc_Q, acc_R);
}

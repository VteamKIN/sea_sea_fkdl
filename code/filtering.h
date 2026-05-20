/*
 * filtering.h
 * 通用滤波器库：卡尔曼 / 中值 / 一阶低通 / 变化率限幅 / 组合滤波
 * Created on: 2026年3月27日
 */

#ifndef CODE_FILTERING_H_
#define CODE_FILTERING_H_

#include "zf_common_headfile.h"

// ===================== 一维卡尔曼滤波器 =====================
typedef struct {
    float x;         // 状态估计值（滤波输出）
    float P;         // 估计协方差
    float Q;         // 过程噪声方差（Q 越大，越信任测量值，响应越快但噪声越多）
    float R;         // 测量噪声方差（R 越大，越信任预测值，滤波越平滑但滞后越大）
    float K;         // 卡尔曼增益
} KalmanFilter;

void    kalman_init(KalmanFilter *kf, float Q, float R, float initial_value);
float   kalman_update(KalmanFilter *kf, float measurement);

// ===================== 3 点中值滤波器 =====================
// 保存最近 3 个采样值，输出中值（去尖峰脉冲）
typedef struct {
    int32 buf[3];    // 环形缓冲 [newest, last, oldest]
    uint8 count;     // 已填充采样数（0~3，不足 3 时直接输出 input）
} MedianFilter3;

void    median3_init(MedianFilter3 *mf);
int32   median3_update(MedianFilter3 *mf, int32 input);

// ===================== 一阶低通滤波器 =====================
// y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
// alpha 越小越平滑，越大响应越快
typedef struct {
    float alpha;     // 滤波系数 (0, 1]
    float last;      // 上次输出
    uint8 inited;    // 首次调用标志
} LowPassFilter;

void    lowpass_init(LowPassFilter *lp, float alpha);
int32   lowpass_update(LowPassFilter *lp, int32 input);

// ===================== 变化率限幅器 =====================
// 每步最大允许变化量 max_delta，超出时裁断
typedef struct {
    float max_delta; // 每步最大允许变化量
    float last;      // 上次输出
    uint8 inited;    // 首次调用标志
} SlewLimiter;

void    slew_init(SlewLimiter *sl, float max_delta);
int32   slew_update(SlewLimiter *sl, int32 input);

// ===================== 组合滤波器 =====================
// 中值 → 变化率限幅 → 一阶低通（三级串联）
// 直接替代 encoder.c 原 _wangyi_ 滤波
typedef struct {
    MedianFilter3 median;
    SlewLimiter   slew;
    LowPassFilter lowpass;
} CompositeFilter;

void    composite_filter_init(CompositeFilter *cf, float alpha, float max_delta);
int32   composite_filter_update(CompositeFilter *cf, int32 input);

// ===================== IMU 陀螺仪三轴卡尔曼 =====================
extern KalmanFilter kf_gyro_x;
extern KalmanFilter kf_gyro_y;
extern KalmanFilter kf_gyro_z;

void gyro_kalman_init(float Q, float R);
void gyro_kalman_update_all(float *gyro_x, float *gyro_y, float *gyro_z);

// ===================== IMU 加速度计三轴卡尔曼 =====================
extern KalmanFilter kf_acc_x;
extern KalmanFilter kf_acc_y;
extern KalmanFilter kf_acc_z;

void acc_kalman_init(float Q, float R);
void acc_kalman_update_all(float *acc_x, float *acc_y, float *acc_z);

// ===================== 一键初始化所有 IMU 滤波器 =====================
void imu_kalman_init_all(float gyro_Q, float gyro_R, float acc_Q, float acc_R);

#endif /* CODE_FILTERING_H_ */


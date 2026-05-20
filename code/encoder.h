/*
 * encoder.h
 * 编码器测速与PID控制模块
 * 主要功能：
 * 1. 编码器数据采集与速度计算
 * 2. PID速度控制
 * 3. 电机转速闭环控制
 *
 * Created on: 2026年1月15日
 *      Author: aaa
 */

#ifndef CODE_ENCODER_H_
#define CODE_ENCODER_H_

#include "zf_common_headfile.h"

// 编码器参数定义
#define ENCODER_PPR         500     // 编码器每转脉冲数
#define SAMPLE_TIME_MS       100     // 采样时间100ms
#define SPEED_MAX          8000    // 最大速度输出
#define SPEED_MIN         -8000    // 最小速度输出
#define INTEGRAL_LIMIT      2000    // 积分限幅

// 编码器方向枚举
typedef enum
{
    ENCODER_LEFT,
    ENCODER_RIGHT,
}encoder_dir_t;

// PID控制器结构体
typedef struct
{
    int16 TargetSpeed;    // 目标转速(rpm)
    int16 ActualSpeed;    // 实际转速(rpm)
    float Kp;             // 比例系数
    float Ki;             // 积分系数
    float Kd;             // 微分系数
    int16 Err;            // 当前误差
    int16 ErrLast;
    int16 ErrPrev;        // 上上一次误差
    int16 Integral;       // 积分累加值
    int16 Output;         // PID输出(PWM占空比)
    int16 OutputLast;
} Encoder_PID_t;

// 全局变量声明
extern volatile int16 encoder_data_l;
extern volatile int16 encoder_data_r;
extern Encoder_PID_t L_PID;
extern Encoder_PID_t R_PID;
extern volatile int16 encoder_conversion_l;
extern volatile int16 encoder_conversion_r;


// 函数声明
void encoder_init(void);                    // 编码器初始化
void Encoder_PID_Init(Encoder_PID_t *pid, float kp, float ki, float kd); // PID初始化
void encoder_interrupt_handler(void);       // 编码器中断处理函数

void L_control(int16 target_speed);         // 左轮速度控制
void R_control(int16 target_speed);         // 右轮速度控制
void motor_control(int16 left_speed, int16 right_speed); // 双轮速度控制
void encoder_read(void);

#endif /* CODE_ENCODER_H_ */

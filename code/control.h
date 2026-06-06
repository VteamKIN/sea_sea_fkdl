/*
 * control.h
 * 控制模块头文件
 * 主要功能：PID控制、偏差计算、电机速度控制
 * Created on: 2026年3月31日
 *      Author: aaa
 */

#ifndef CODE_CONTROL_H_
#define CODE_CONTROL_H_

#include "zf_common_headfile.h"

// ============================================================
// PID 控制参数
// ============================================================

extern int16 control_base_speed;            // 基础速度
#define I_LIMIT                    1000     // 积分项限幅值
#define OUTPUT_LIMIT               8000     // 输出限幅值
#define CONTROL_SPEED_MAX          8000     // 控制模块最大速度

// 堵转保护停车
#define STALL_MAX_ENCODER          3        // 编码器绝对值和低于此值视为堵转
#define STALL_STOP_FRAMES          20       // 连续堵转帧数达到此值则停车

// 高度保护（Y轴垂直速度）
#define HEIGHT_VEL_THRESHOLD       0.10f    // 垂直速度阈值 (m/s)
#define HEIGHT_STOP_FRAMES         2        // 连续超阈值帧数后停车

// 撞墙保护（IMU水平速度）
#define WALL_HOR_VEL_MIN           0.02f    // 水平速度低于此值视为未移动 (m/s)
#define WALL_ENCODER_MIN           10       // 编码器和高于此值说明轮子在转
#define WALL_STOP_FRAMES           30       // 连续"轮转但不动"帧数后停车

// 丢线保护
#define LINE_LOST_GRACE_FRAMES     5        // 丢线后维持偏差的容忍帧数，之后停车

// ============================================================
// 偏差计算参数
// ============================================================

#define HIST_PEAK_RADIUS           3        // 直方图众数中心计算的邻域半径（像素）



typedef enum
{
    SPEED_CHANGE_LINEAR,        // 0 线性：固定步长 speed_change_step
    SPEED_CHANGE_EXP_MIN,       // 1 指数 + 最小步长封底：max(|delta|*alpha, min_step)
    SPEED_CHANGE_SQRT,          // 2 平方根步长：k * sqrt(|delta|)
    SPEED_CHANGE_POWER,         // 3 幂律步长：k * |delta|^p（p 可调 0~1）
    SPEED_CHANGE_TWO_STAGE,     // 4 两段式：远 alpha_far，近 alpha_near
} change_mode_enum;

// ============================================================
// 全局变量声明（extern）
// ============================================================

// PID参数
extern float kp;                  // 比例系数
extern float ki;                  // 积分系数
extern float kd;                  // 微分系数
extern float curve_kp;            // 弯道比例系数
extern float curve_ki;            // 弯道积分系数
extern float curve_kd;            // 弯道微分系数
extern int16 pursuit_lookahead;    // Pure Pursuit 前瞻点索引
extern int16 pursuit_inward_bias;  // Pure Pursuit 单边循线内偏量
extern int16 error_limit;          // 图像偏差限幅

// PID 控制变量
extern vint16 P;                  // 比例项
extern vint16 I;                  // 积分项（限幅 ±I_LIMIT）
extern vint16 D;                  // 微分项
extern vint16 i_max;              // 积分最大值

extern vint16 error_image;        // 图像偏差（像素，PID 输入，±50 限幅）
extern vint16 error_left;         // 循左边线偏差（外部观察接口，calc_error_image 不更新）
extern vint16 error_right;        // 循右边线偏差（外部观察接口，calc_error_image 不更新）
extern vint16 last_error_image;   // 上一帧 error_image（PID 微分项 D 使用）
extern vint16 last_error;         // 上一帧 error_image（calc_error_image 末尾平滑滤波使用）

extern vint16 output;             // PID 输出（速度偏移量，限幅 ±OUTPUT_LIMIT）

// 电机速度变量
extern int16 straight_base_speed;  // 直道基础速度
extern int16 curve_base_speed;     // 弯道基础速度
extern vint16 left_speed;         // 左轮速度
extern vint16 right_speed;        // 右轮速度

// 运行状态
extern vint8 car_running;         // 1:运行  0:停车
extern uint8 line_lost_protect_enable; // 1:启用直道丢线停车
extern float vert_vel;            // Y轴垂直速度 (m/s)，高度保护用

extern uint16 process_time;                 // control_process 一次调用耗时（μs，调试观察用）
extern volatile uint16 dir_advance_count;   // dir_count 累计自增次数（供上层调试统计）

// speed_change 调参
extern int16 speed_change_step;         // LINEAR:    每帧步长（默认 10）
extern float speed_change_alpha;        // EXP_MIN:   指数系数 0~1（默认 0.1）
extern int16 speed_change_min_step;     // EXP_MIN:   最小步长封底（默认 1）
extern float speed_change_sqrt_k;       // SQRT:      平方根系数（默认 2.0）
extern float speed_change_power_k;      // POWER:     幂律系数（默认 2.0）
extern float speed_change_power_p;      // POWER:     幂律指数 0~1（默认 0.5）
extern int16 speed_change_threshold;    // TWO_STAGE: 远近分段阈值（默认 200）
extern float speed_change_alpha_far;    // TWO_STAGE: 远端指数系数（默认 0.2）
extern float speed_change_alpha_near;   // TWO_STAGE: 近端指数系数（默认 0.05）


// ============================================================
// 函数声明
// ============================================================

// 计算图像偏差
void calc_error_image(void);

// 图像 PID 控制计算：error_image输入，更新output
void image_pid_out(void);

// 控制处理主函数
void control_process(void);

//重置PID控制器
void control_pid_reset(void);

// 重置丢线保护计数器
void control_line_lost_reset(void);

// 速度切换：每帧调用，按 mode 将 cur_speed 朝 tar_speed 过渡，写回 control_base_speed
void speed_change(int16 cur_speed, int16 tar_speed, change_mode_enum mode);

#endif /* CODE_CONTROL_H_ */

/*
 * imu.h
 * IMU660RA 陀螺仪驱动模块 - 角度测量与角度环控制
 * Created on: 2026年1月16日
 * Author: aaa
 */

#ifndef CODE_IMU_H_
#define CODE_IMU_H_

#include "zf_common_headfile.h"

//------------------------------------------------------------------------------------------------------------------
// 错误码定义
//------------------------------------------------------------------------------------------------------------------
#define imu_mode_error 0xFFFF    // 模式错误
#define imu_dir_error  0x0000    // 方向错误

//------------------------------------------------------------------------------------------------------------------
// 阻塞转弯函数的超时保护（防死锁）
// 当 IMU 未启用（init.c 中 my_imu_init 被注释时）yaw_angle 永远为 0，
// turn_*_90 的 while 循环永远等不到 angle_is_reached，会永久卡死主线程。
// 本宏定义等待上限，到期直接返回，保证系统不会被冻结。
//------------------------------------------------------------------------------------------------------------------
#define IMU_TURN_TIMEOUT_MS        2000    // turn_*_90 阻塞等待上限 (ms)

//------------------------------------------------------------------------------------------------------------------
// IMU 原始数据 (int16)
//------------------------------------------------------------------------------------------------------------------
extern int16 imu_gyro_data_x, imu_gyro_data_y, imu_gyro_data_z;  // 三轴陀螺仪原始数据
extern int16 imu_acc_data_x, imu_acc_data_y, imu_acc_data_z;     // 三轴加速度计原始数据

//------------------------------------------------------------------------------------------------------------------
// IMU 浮点数据 (经过转换单位)
//------------------------------------------------------------------------------------------------------------------
extern float imu_acc_x_f, imu_acc_y_f, imu_acc_z_f;   // 加速度计数据 (g)
extern float imu_gyro_x_f, imu_gyro_y_f, imu_gyro_z_f; // 陀螺仪数据 (deg/s)
extern float yaw_angle;                                // 航向角 (deg), 范围 [-180, 180]
extern float gyro_offset;                            // 陀螺仪Z轴零偏 (deg/s)
extern float angle_out;

//------------------------------------------------------------------------------------------------------------------
// IMU速度控制变量
//------------------------------------------------------------------------------------------------------------------
extern int16 left_speed_imu;    // 左轮IMU目标速度
extern int16 right_speed_imu;   // 右轮IMU目标速度

//------------------------------------------------------------------------------------------------------------------
// 枚举类型定义
//------------------------------------------------------------------------------------------------------------------
typedef enum {
    mode_acc = 0,   // 加速度模式
    mode_gyro = 1,  // 陀螺仪模式
} imu_mode_enum;

typedef enum {
    dir_x = 0,  // X轴
    dir_y = 1,  // Y轴
    dir_z = 2,  // Z轴
} imu_dir_enum;

//------------------------------------------------------------------------------------------------------------------
// 角度环参数
//------------------------------------------------------------------------------------------------------------------
extern float angle_kp;              // 比例系数
extern float angle_kd;              // 微分系数
extern float angle_output;          // 角度环输出值
extern float angle_error;           // 当前角度误差 (deg)
extern float angle_output_max;      // 输出限幅
extern uint8_t angle_loop_enable;   // 角度环使能标志: 0=禁用, 1=使能
extern float target_angle;          // 目标角度 (deg)

//------------------------------------------------------------------------------------------------------------------
// 基础函数
//------------------------------------------------------------------------------------------------------------------

/**
 * @brief  角度归一化到 [-180, 180] 度
 * @param  angle 输入角度 (deg)
 * @return 归一化后的角度 (deg)
 */
float normalize_angle(float angle);

/**
 * @brief  陀螺仪零偏校准 (上电静止状态下调用)
 * @note   校准过程约500ms，期间需保持静止
 */
void gyro_calib(void);

/**
 * @brief  航向角积分更新
 * @note   必须在固定周期中断中调用 (如2ms定时中断)
 */
void gyro_update(void);

//------------------------------------------------------------------------------------------------------------------
// 角度环控制函数
//------------------------------------------------------------------------------------------------------------------

/**
 * @brief  初始化角度环参数
 * @param  kp 比例系数
 * @param  kd 微分系数
 */
void angle_loop_init(float kp, float kd);

/**
 * @brief  设置目标角度
 * @param  target 目标角度 (deg), 会自动归一化到 [-180, 180]
 */
void angle_set_target(float target);

/**
 * @brief  角度环PD控制计算
 * @return 角度环输出值 (用于差速控制的PWM调整量)
 * @note   需在固定周期中断中调用
 */
float angle_loop_update(void);

/**
 * @brief  使能或禁用角度环
 * @param  enable 0=禁用, 1=使能
 */
void angle_loop_enable_set(uint8_t enable);

/**
 * @brief  重置角度环状态 (角度、误差、输出清零)
 */
void angle_loop_reset(void);

/**
 * @brief  相对角度转弯
 * @param  delta_angle 相对转动角度 (deg), 正值左转, 负值右转
 * @note   在当前角度基础上叠加delta_angle作为目标角度
 */
void angle_turn_relative(float delta_angle);

/**
 * @brief  绝对角度转弯
 * @param  absolute_angle 目标绝对角度 (deg)
 */
void angle_turn_absolute(float absolute_angle);

/**
 * @brief  判断是否到达目标角度
 * @param  threshold 允许的角度误差阈值 (deg)
 * @return 1=已到达目标, 0=未到达
 */
uint8_t angle_is_reached(float threshold);

/**
 * @brief  在当前角度基础上左转90°
 */
void turn_left_90(void);
void turn_right_90(void);
/**
 * @brief  转到90°
 */
void turn_left_absolute_90(void);
void turn_right_absolute_90(void);

void my_imu_init(void);
void imu_control(void);  // IMU速度控制计算


/**
 * @brief  非阻塞式转弯函数
 * @note   启动转弯后立即返回，通过 is_turn_complete() 检查完成状态
 */
void start_turn_left_90(void);
void start_turn_right_90(void);
uint8_t is_turn_complete(void);



#endif /* CODE_IMU_H_ */

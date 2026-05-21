/*
 * motor.h
 * 电机控制模块头文件
 * 主要功能：电机初始化、速度控制、死区时间配置
 * Created on: 2026年1月15日
 *      Author: aaa
 */

#ifndef CODE_MOTOR_H_
#define CODE_MOTOR_H_

#include "zf_common_headfile.h"

// 电机参数定义
#define speed_max 8000


// 电机PWM通道定义
#define L_CH ATOM0_CH1_P10_1
#define L_CL ATOM0_CH3_P10_3
#define R_CH ATOM0_CH4_P02_4
#define R_CL ATOM0_CH5_P02_5

//#define L_CH ATOM0_CH1_P10_1
//#define L_CL ATOM0_CH3_P10_3
//#define R_CH ATOM0_CH4_P02_4
//#define R_CL ATOM0_CH5_P02_5

#define F_CH ATOM1_CH2_P10_5
#define F_CL ATOM0_CH6_P02_6

void motor_init(void);
void fan_init(void);
void motor_left(int speed);
void motor_right(int speed);
void motor_set(int left_speed, int right_speed);
void fan_set(int fan_speed);
void motor_stop(void);
void fan_slow_stop(void);
void motor_close(void);
#endif /* CODE_MOTOR_H_ */

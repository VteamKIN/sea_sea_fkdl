/*
 * motor.c
 * 电机驱动
 *  Created on: 2026年1月15日
 *      Author: aaa
 */

#include "zf_common_headfile.h"

#pragma section all "cpu0_dsram"

int speed;

#pragma section all restore


void motor_init(void)
{

    pwm_init(L_CH, 17*1000, 0);
    pwm_init(L_CL, 17*1000, 0);
    pwm_init(R_CH, 17*1000, 0);
    pwm_init(R_CL, 17*1000, 0);

}

void fan_init(void)
{
    pwm_init(F_CH, 17*1000, 0);
    pwm_init(F_CL, 17*1000, 0);
}




void motor_left(int speed)//speed是占空比
{

    if (speed >= 0){
        pwm_set_duty(L_CH, speed);
        pwm_set_duty(L_CL, 10000);
    }
    else{
        pwm_set_duty(L_CH, -speed);
        pwm_set_duty(L_CL, 0);
    }
}

void motor_right(int speed)
{

    if (speed >= 0){
        pwm_set_duty(R_CH, speed);
        pwm_set_duty(R_CL, 0);
    }
    else{
        pwm_set_duty(R_CH, -speed);
        pwm_set_duty(R_CL, 10000);
    }
}

/*void motor_set(int16 left_speed, int16 right_speed)
{
    L_control(left_speed);
    R_control(right_speed);
}
*/


void motor_set(int left_speed, int right_speed)
{
    motor_left(left_speed);
    motor_right(right_speed);
}

void fan_set(int fan_speed)
{
    if (fan_speed >= 0)
    {
            pwm_set_duty(F_CH, fan_speed);
            pwm_set_duty(F_CL, 0);
    }
    else
    {
            pwm_set_duty(F_CH, -fan_speed);
            pwm_set_duty(F_CL, 0);
    }
}

void fan_slow_stop(void)
{
    fan_set(0);
}
void motor_stop(void)
{
    motor_set(0, 0);

}

void motor_close(void)
{
    pwm_all_channel_close();
}

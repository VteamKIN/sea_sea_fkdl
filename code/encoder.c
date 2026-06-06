/*
 * encoder.c
 * 编码器测速与PID控制模块
 * 主要功能：
 * 1. 编码器数据采集与速度计算
 * 2. PID速度控制
 * 3. 电机转速闭环控制
 *
 * Created on: 2026年1月15日
 *      Author: aaa
 */

#include "zf_common_headfile.h"

// 编码器硬件定义
#define ENCODER_LEFT_TIMER      (TIM2_ENCODER)           // 左编码器定时器
#define ENCODER_LEFT_A_PIN      (TIM2_ENCODER_CH1_P33_7) // 左编码器A相引脚
#define ENCODER_LEFT_B_PIN      (TIM2_ENCODER_CH2_P33_6) // 左编码器B相引脚

#define ENCODER_RIGHT_TIMER     (TIM4_ENCODER)           // 右编码器定时器
#define ENCODER_RIGHT_A_PIN     (TIM4_ENCODER_CH1_P02_8) // 右编码器A相引脚
#define ENCODER_RIGHT_B_PIN     (TIM4_ENCODER_CH2_P00_9) // 右编码器B相引脚

#define encoder_time    2                             //编码器获取数据的周期

// 全局变量定义（使用volatile保证中断可见性）
volatile int16 encoder_data_l = 0;
volatile int16 encoder_data_r = 0;
volatile int16 encoder_conversion_l = 0;
volatile int16 encoder_conversion_r = 0;

/*
#define LEFT_PID_KP_DEFAULT   150    // 左轮默认Kp = 1.5
#define LEFT_PID_KI_DEFAULT   20     // 左轮默认Ki = 0.2
#define LEFT_PID_KD_DEFAULT   5      // 左轮默认Kd = 0.05
*/

// PID控制器实现
Encoder_PID_t L_PID = {
    .TargetSpeed = 0,
    .ActualSpeed = 0,
    .Kp = 0,     // 默认PID参数，需要根据实际调试放大100倍
    .Ki = 0,
    .Kd = 0,
    .Err = 0,
    .ErrLast = 0,
    .ErrPrev = 0,
    .Integral = 0,
    .Output = 0,
    .OutputLast = 0
};

Encoder_PID_t R_PID = {
    .TargetSpeed = 0,
    .ActualSpeed = 0,
    .Kp = 0,
    .Ki = 0,
    .Kd = 0,
    .Err = 0,
    .ErrLast = 0,
    .ErrPrev = 0,
    .Integral = 0,
    .Output = 0,
    .OutputLast = 0
};

// 私有函数声明
static int16 constrain(int16 value, int16 min_val, int16 max_val);
//static int16 pulse_to_rpm(int16 pulse_count);

//--------------------------------------------------------------------------------------
// 函数：编码器初始化
// 功能：初始化编码器硬件和定时器中断
//--------------------------------------------------------------------------------------
void encoder_init(void)
{
    // 初始化左编码器（方向编码模式）
    encoder_dir_init(ENCODER_LEFT_TIMER, ENCODER_LEFT_A_PIN, ENCODER_LEFT_B_PIN);
    
    // 初始化右编码器（方向编码模式）
    encoder_dir_init(ENCODER_RIGHT_TIMER, ENCODER_RIGHT_A_PIN, ENCODER_RIGHT_B_PIN);


    

    // 初始化PID控制器
    Encoder_PID_Init(&L_PID, 3,0.32,0.15);
    Encoder_PID_Init(&R_PID,3,0.28,0.1);
    
    pit_ms_init(CCU60_CH0, encoder_time);
    

}

//--------------------------------------------------------------------------------------
// 函数：PID控制器初始化
// 参数：pid - PID控制器指针
//       kp, ki, kd - PID参数
//--------------------------------------------------------------------------------------
void Encoder_PID_Init(Encoder_PID_t *pid, float kp, float ki, float kd)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->TargetSpeed = 0;
    pid->ActualSpeed = 0;
    pid->Err = 0;
    pid->ErrLast = 0;
    pid->Integral = 0;
    pid->Output = 0;
}



//--------------------------------------------------------------------------------------
// 函数：数值限幅
// 参数：value - 输入值
//       min_val, max_val - 限幅范围
// 返回：限幅后的值
//--------------------------------------------------------------------------------------
static int16 constrain(int16 value, int16 min_val, int16 max_val)
{
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}



static void Encoder_PID_Reset_State(Encoder_PID_t *pid)
{
    pid->TargetSpeed = 0;
    pid->ActualSpeed = 0;
    pid->Err = 0;
    pid->ErrLast = 0;
    pid->ErrPrev = 0;
    pid->Integral = 0;
    pid->Output = 0;
    pid->OutputLast = 0;
}

void encoder_speed_pid_reset(void)
{
    Encoder_PID_Reset_State(&L_PID);
    Encoder_PID_Reset_State(&R_PID);
}
//--------------------------------------------------------------------------------------
// 函数：左轮速度控制
// 参数：target_speed - 目标转速(rpm)
//--------------------------------------------------------------------------------------
void L_control(int16 target_speed)
{
    // 设置目标速度
       L_PID.TargetSpeed = target_speed;

       // 计算当前误差
       L_PID.Err = L_PID.TargetSpeed - L_PID.ActualSpeed;

       // 增量式PID核心公式：ΔOutput = Kp*(Err-ErrLast) + Ki*Err + Kd*(Err-2*ErrLast+ErrPrev)
       float delta_Output =
           (L_PID.Kp * (L_PID.Err - L_PID.ErrLast)) +
           (L_PID.Ki * L_PID.Err) +
           (L_PID.Kd * (L_PID.Err - 2 * L_PID.ErrLast + L_PID.ErrPrev));

       L_PID.Output = L_PID.OutputLast + (int16)delta_Output;

       // 输出限幅（防止溢出）
       L_PID.Output = constrain(L_PID.Output, SPEED_MIN, SPEED_MAX);

       // 更新误差历史（为下一次计算做准备）
       L_PID.ErrPrev = L_PID.ErrLast;
       L_PID.ErrLast = L_PID.Err;
       L_PID.OutputLast = L_PID.Output;

       // 驱动左轮电机
       motor_left(L_PID.Output);
}

//--------------------------------------------------------------------------------------
// 函数：右轮速度控制
// 参数：target_speed - 目标转速(rpm)
//--------------------------------------------------------------------------------------
void R_control(int16 target_speed)
{
    // 设置右轮目标速度
        R_PID.TargetSpeed = target_speed;

        // 计算右轮当前误差
        R_PID.Err = R_PID.TargetSpeed - R_PID.ActualSpeed;

        // 增量式PID核心计算公式
        float delta_Output =
            (R_PID.Kp * (R_PID.Err - R_PID.ErrLast)) +
            (R_PID.Ki * R_PID.Err) +
            (R_PID.Kd * (R_PID.Err - 2 * R_PID.ErrLast + R_PID.ErrPrev));

        R_PID.Output = R_PID.OutputLast + (int16)delta_Output;

        // 输出限幅保护（和左轮保持相同限制范围）
        R_PID.Output = constrain(R_PID.Output, SPEED_MIN, SPEED_MAX);

        // 更新历史误差值，为下一次PID计算做准备
        R_PID.ErrPrev = R_PID.ErrLast;
        R_PID.ErrLast = R_PID.Err;
        R_PID.OutputLast = R_PID.Output;

        // 驱动右轮电机执行输出
        motor_right(R_PID.Output);
}

//--------------------------------------------------------------------------------------
// 函数：双轮速度控制
// 参数：left_speed - 左轮目标转速(rpm)
//       right_speed - 右轮目标转速(rpm)
//--------------------------------------------------------------------------------------
void motor_control(int16 left_speed, int16 right_speed)
{
    if (left_speed == 0 && right_speed == 0)
    {
        encoder_speed_pid_reset();
        motor_set(0, 0);
        return;
    }

    L_control(left_speed);
    R_control(right_speed);
}

// 编码器滤波器实例（左右轮独立，避免状态交叉污染）
// alpha=0.99 与原 _wangyi_ 一致，max_delta=8000 与原 max_diff 一致
static CompositeFilter encoder_filter_l;
static CompositeFilter encoder_filter_r;
static uint8 encoder_filter_inited = 0;

/**
 * @brief 编码器读取数据
 * @param void
 */
void encoder_read(void)
{
    if (!encoder_filter_inited)
    {
        composite_filter_init(&encoder_filter_l, 0.99f, 8000.0f);
        composite_filter_init(&encoder_filter_r, 0.99f, 8000.0f);
        encoder_filter_inited = 1;
    }

    encoder_data_l = encoder_get_count(TIM2_ENCODER);                              // 获取编码器计数
    encoder_clear_count(TIM2_ENCODER);                                             // 清空编码器计数
    encoder_conversion_l=encoder_data_l*38;
    L_PID.ActualSpeed=composite_filter_update(&encoder_filter_l, encoder_conversion_l);

    encoder_data_r = -encoder_get_count(TIM4_ENCODER);                              // 获取编码器计数
    encoder_clear_count(TIM4_ENCODER);                                             // 清空编码器计数
    encoder_conversion_r=encoder_data_r*38;
    R_PID.ActualSpeed=composite_filter_update(&encoder_filter_r, encoder_conversion_r);
}

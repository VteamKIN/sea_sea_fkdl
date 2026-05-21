/*
 * control.c
 * 控制模块实现
 * 主要功能：PID控制、偏差计算、电机速度控制
 * Created on: 2026年3月31日
 *      Author: aaa
 *
 * ============================================================
 * 模块架构
 * ============================================================
 *  1. 偏差计算    — 基于边界数据计算图像偏差
 *  2. PID 控制    — 比例、积分、微分计算（两套PID自动切换）
 *  3. 电机控制    — 根据PID输出控制左右电机速度
 *  4. 弯道处理    — 航向角辅助判断弯道完成
 * ============================================================
 */

#include "zf_common_headfile.h"
#include "imu.h"

#pragma section all "cpu0_dsram"

// ============================================================
// 全局变量
// ============================================================

// PID 参数
float kp = 20.0f;                           // PID 比例系数
float ki = 0.00f;                           // PID 积分系数
float kd = 0.0f;                            // PID 微分系数

// PID 控制变量
vint16 P = 0;                               // 比例项
vint16 I = 0;                               // 积分项（限幅 ±I_LIMIT）
vint16 D = 0;                               // 微分项

// 偏差变量
vint16 error_image = 0;                     // 图像偏差（像素，PID 输入，±50 限幅）
vint16 last_error_image = 0;                // 上一帧 error_image（PID 微分项 D = 当前 - 上一帧）
vint16 last_error = 0;                      // 上一帧 error_image（calc_error_image 末尾平滑滤波用）

// PID 输出
static vint16 last_output = 0;              // 上一帧 PID 输出
vint16 output = 0;                          // PID 输出（速度偏移量，限幅 ±OUTPUT_LIMIT）

// 电机速度
int16 control_base_speed = 1200;             //控制模块基础速度
vint16 left_speed = 0;                      // 左轮速度
vint16 right_speed = 0;                     // 右轮速度

// Pure Pursuit 参数
int16  pursuit_lookahead = 40;              // 前瞻取边线第 N 个点（索引，0=底部最近）
int16  pursuit_inward_bias = 3;             // 弯道内偏量（像素，循单边时朝内侧偏置）

// speed_change 调参
int16 speed_change_step       = 10;     // LINEAR    每帧步长
float speed_change_alpha      = 0.1f;   // EXP_MIN   指数系数
int16 speed_change_min_step   = 1;      // EXP_MIN   最小步长封底
float speed_change_sqrt_k     = 2.0f;   // SQRT      平方根系数
float speed_change_power_k    = 2.0f;   // POWER     幂律系数
float speed_change_power_p    = 0.5f;   // POWER     幂律指数
int16 speed_change_threshold  = 200;    // TWO_STAGE 分段阈值
float speed_change_alpha_far  = 0.2f;   // TWO_STAGE 远端系数
float speed_change_alpha_near = 0.05f;  // TWO_STAGE 近端系数

// 运行状态
vint8 car_running = 1;                      // 1:运行  0:停车

// 调试 / 统计
uint16 process_time = 0;                    // control_process 一次调用耗时（μs，调试观察用）
volatile uint16 dir_advance_count = 0;      // dir_count 累计自增次数（供上层调试统计）

#pragma section all restore

//===================================================================================================================
// 函数简介     对称限幅
// 参数说明     value   输入值
// 参数说明     limit   限幅绝对值
// 返回参数     int16   裁断到 [-limit, +limit]
// 备注信息
//===================================================================================================================
static int16 int_limit(int16 value, int16 limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

//===================================================================================================================
// 函数简介     取边线第 idx 个点（前瞻目标）
// 参数说明     edge       边线点数组 [i][0]=x [i][1]=y，索引 0=底部（近车）
// 参数说明     count      边线点数
// 参数说明     idx        目标点索引
// 参数说明     out_x,out_y 输出目标点
// 返回参数     uint8      1=找到目标点（点数不足时返回最末点） 0=边线为空
// 备注信息     索引超出 count-1 时返回最末点（仍提供方向信息）
//===================================================================================================================
#define PURSUIT_MIN_POINTS    3
static uint8 pursuit_walk_edge(const int16 edge[][2], int count, int idx,
                               int16 *out_x, int16 *out_y)
{
    if (count < PURSUIT_MIN_POINTS) return 0;

    if (idx >= count) idx = count - 1;
    if (idx < 0)      idx = 0;

    *out_x = edge[idx][0];
    *out_y = edge[idx][1];
    return 1;
}

//===================================================================================================================
// 函数简介     计算图像偏差 error_image（PID 输入）— Pure Pursuit
// 参数说明     void
// 返回参数     void（结果写入全局 error_image）
// 备注信息     取边线第 pursuit_lookahead 个点为目标点 Tx，error = Tx - 中心
//             直道循双边中点，左/右弯循对应边线 ± (width_half - inward_bias)
//             两边都丢线时 error 衰减
//===================================================================================================================
void calc_error_image(void)
{
    int16 width_full = (road_width_avg > 0) ? (int16)road_width_avg : 3;
    int16 width_half = width_full / 2;
    int16 inward_off = width_half - pursuit_inward_bias;
    if (inward_off < 0) inward_off = 0;

    int16 Lx = 0, Ly = 0, Rx = 0, Ry = 0;
    uint8 left_ok  = pursuit_walk_edge(left_edge,  left_edge_count,
                                       pursuit_lookahead, &Lx, &Ly);
    uint8 right_ok = pursuit_walk_edge(right_edge, right_edge_count,
                                       pursuit_lookahead, &Rx, &Ry);

    int16 target_x = WARP_IMAGE_W / 2;
    uint8 target_valid = 0;

    switch (road_type)
    {
        case straight:
            if (left_ok && right_ok)
            {
                target_x = (int16)((Lx + Rx) / 2);
                target_valid = 1;
            }
            break;

        case left:
            if (left_ok)
            {
                target_x = Lx + inward_off;
                target_valid = 1;
            }
            break;

        case right:
            if (right_ok)
            {
                target_x = Rx - inward_off;
                target_valid = 1;
            }
            break;
    }

    if (!target_valid)
    {
        // 丢线 → 维持上一帧偏差不变
        return;
    }

    // clamp 目标 x
    if (target_x < 0) target_x = 0;
    if (target_x > WARP_IMAGE_W - 1) target_x = WARP_IMAGE_W - 1;

    int16 new_error = target_x - WARP_IMAGE_W / 2;

    // 限幅
    if (new_error > 50) new_error = 50;
    if (new_error < -50) new_error = -50;

    // 平滑滤波
    error_image = (int16)(0.9f * (float)new_error + 0.1f * (float)last_error);
    last_error  = error_image;
}


//===================================================================================================================
// 函数简介     图像 PID 控制计算
// 参数说明     void
// 返回参数     void（结果写入全局 output）
// 备注信息     输入 error_image，I 项 ±I_LIMIT 限幅，output ±OUTPUT_LIMIT 限幅
//===================================================================================================================
void image_pid_out(void)
{
    last_output = output;
    P = error_image;

    I += error_image;
    I = int_limit(I, I_LIMIT);

    D = error_image - last_error_image;

    output = kp * P + ki * I + kd * D;
    output = int_limit(output, OUTPUT_LIMIT);

    last_error_image = error_image;
}


//===================================================================================================================
// 函数简介     控制处理主函数（每帧调用）
// 参数说明     void
// 返回参数     void
// 备注信息     停车判断 → dir_advance 事件响应 → calc_error → PID → motor_control
//===================================================================================================================
void control_process(void)
{
    uint16 time1 = system_getval_us();

    if (img_process_time == 0)
    {
        car_running = 0;
        motor_control(0, 0);
        return;
    }

    if (dir_count >= road_num)
    {
        car_running = 0;
        motor_control(0, 0);
        return;
    }

    // 检查是否需要停车
    if (choose[dir_count] == -1)
    {
        car_running = 0;
        motor_control(0, 0);
        return;
    }
    // 停车
    if (!car_running)
    {
        motor_control(0, 0);
        fan_slow_stop();
        return;
    }

    // 计算图像偏差
    calc_error_image();


    // PID输出
    image_pid_out();


    // 电机控制
    motor_control(control_base_speed + output, control_base_speed - output);

    dir_advance_pending = 0;
    uint16 time2 = system_getval_us();
    process_time = time2 - time1;
}

//===================================================================================================================
// 函数简介     重置 PID 内部状态
// 返回参数     void
// 备注信息     弯道完成时调用，防止上一段误差污染下一段控制
//=====================================================================================================================
void control_pid_reset(void)
{
    P = 0;
    I = 0;
    D = 0;
    error_image = 0;
    last_error_image = 0;
    last_error = 0;
    output = 0;
    last_output = 0;
}

//===================================================================================================================
// 函数简介     速度切换（每帧调用，非阻塞）：将 cur_speed 朝 tar_speed 过渡，写回 control_base_speed
// 参数说明     cur_speed  当前基础速度（通常传入 control_base_speed）
// 参数说明     tar_speed  目标速度
// 参数说明     mode       切换模式：
//                         LINEAR     固定步长
//                         EXP_MIN    指数 + 最小步长封底（起步大、后面封底）
//                         SQRT       平方根步长（起步较大，过渡柔和）
//                         POWER      幂律步长 k*|delta|^p（p 可调猛度）
//                         TWO_STAGE  两段式（远快近慢）
// 返回参数     void（结果写入全局 control_base_speed）
// 使用示例     speed_change(control_base_speed, 1200, SPEED_CHANGE_EXP_MIN);
// 备注信息     所有模式都保证有限帧到达目标、防越过、受 CONTROL_SPEED_MAX 限幅
//===================================================================================================================
void speed_change(int16 cur_speed, int16 tar_speed, change_mode_enum mode)
{
    int32 delta = (int32)tar_speed - (int32)cur_speed;
    if (delta == 0)
    {
        control_base_speed = tar_speed;
        return;
    }

    int32 sign = (delta > 0) ? 1 : -1;
    int32 abs_delta = (delta > 0) ? delta : -delta;
    int32 step = 0;   // 绝对步长，最后乘 sign

    switch (mode)
    {
        case SPEED_CHANGE_LINEAR:
        {
            step = (speed_change_step > 0) ? speed_change_step : abs_delta;
            break;
        }

        case SPEED_CHANGE_EXP_MIN:
        {
            int32 exp_step = (int32)((float)abs_delta * speed_change_alpha);
            int32 min_step = (speed_change_min_step > 0) ? speed_change_min_step : 1;
            step = (exp_step > min_step) ? exp_step : min_step;
            break;
        }

        case SPEED_CHANGE_SQRT:
        {
            float s = speed_change_sqrt_k * sqrtf((float)abs_delta);
            step = (int32)s;
            if (step < 1) step = 1;                 // 防停滞
            break;
        }

        case SPEED_CHANGE_POWER:
        {
            float s = speed_change_power_k * powf((float)abs_delta, speed_change_power_p);
            step = (int32)s;
            if (step < 1) step = 1;                 // 防停滞
            break;
        }

        case SPEED_CHANGE_TWO_STAGE:
        {
            float a = (abs_delta > speed_change_threshold)
                      ? speed_change_alpha_far
                      : speed_change_alpha_near;
            step = (int32)((float)abs_delta * a);
            if (step < 1) step = 1;                 // 防停滞
            break;
        }

        default:
            step = abs_delta;                       // 未知模式直接跳到目标
            break;
    }

    // 防越过目标
    if (step > abs_delta) step = abs_delta;

    int32 n = (int32)cur_speed + sign * step;

    // 全局限幅
    if (n >  CONTROL_SPEED_MAX) n =  CONTROL_SPEED_MAX;
    if (n < -CONTROL_SPEED_MAX) n = -CONTROL_SPEED_MAX;

    control_base_speed = (int16)n;
}











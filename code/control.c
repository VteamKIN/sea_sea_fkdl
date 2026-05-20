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
vint16 i_max = 0;                           // 积分最大值

// 偏差变量
vint16 error_image = 0;                     // 图像偏差（像素，PID 输入，±50 限幅）
vint16 error_left = 0;                      // 循左边线偏差（外部观察接口，calc_error_image 不更新）
vint16 error_right = 0;                     // 循右边线偏差（外部观察接口，calc_error_image 不更新）
vint16 last_error_image = 0;                // 上一帧 error_image（PID 微分项 D = 当前 - 上一帧）
vint16 last_error = 0;                      // 上一帧 error_image（calc_error_image 末尾平滑滤波用）

// PID 输出
static vint16 last_output = 0;              // 上一帧 PID 输出
vint16 output = 0;                          // PID 输出（速度偏移量，限幅 ±OUTPUT_LIMIT）

// 电机速度
int16 control_base_speed = 1000;             //控制模块基础速度
vint16 left_speed = 0;                      // 左轮速度
vint16 right_speed = 0;                     // 右轮速度

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
// 函数简介     计算直方图中位数
// 参数说明     hist    直方图数组
// 参数说明     total   权重总和
// 返回参数     int16   中位数对应的 x 坐标
// 备注信息     从 x=0 累加过 total/2 即中位数，适用于双峰分布（双边线合并）
//===================================================================================================================
static int16 get_hist_median(const uint16 *hist, int total)
{
    int mid = total / 2;
    int acc = 0;
    for (int x = 0; x < WARP_IMAGE_W; x++)
    {
        acc += hist[x];
        if (acc > mid)
        {
            return (int16)x;
        }
    }
    return (int16)(WARP_IMAGE_W / 2);
}

//===================================================================================================================
// 函数简介     计算直方图主峰中心（众数邻域加权重心）
// 参数说明     hist    直方图数组
// 参数说明     total   权重总和
// 返回参数     int16   主峰中心 x 坐标
// 备注信息     适用单峰分布，先找 argmax 再在 ±HIST_PEAK_RADIUS 邻域加权求重心，抗离群点
//===================================================================================================================
static int16 get_hist_mode_center(const uint16 *hist, int total)
{
    if (total <= 0) return (int16)(WARP_IMAGE_W / 2);

    // 扫描最大值位置
    int max_val = 0;
    int peak_x = WARP_IMAGE_W / 2;
    for (int x = 0; x < WARP_IMAGE_W; x++)
    {
        if (hist[x] > max_val)
        {
            max_val = hist[x];
            peak_x = x;
        }
    }

    // 主峰邻域加权重心，平滑离群点
    int x_lo = peak_x - HIST_PEAK_RADIUS;
    int x_hi = peak_x + HIST_PEAK_RADIUS;
    if (x_lo < 0) x_lo = 0;
    if (x_hi >= WARP_IMAGE_W) x_hi = WARP_IMAGE_W - 1;

    int sum_xw = 0;
    int sum_w = 0;
    for (int x = x_lo; x <= x_hi; x++)
    {
        sum_xw += x * hist[x];
        sum_w  += hist[x];
    }

    if (sum_w > 0)
        return (int16)(sum_xw / sum_w);
    return (int16)peak_x;
}

//===================================================================================================================
// 函数简介     扫描 edge_list 并累计加权直方图
// 参数说明     edge / edge_count              边线点数组及点数
// 参数说明     y_start / y_end                采样区 y 范围 [含, 不含)
// 参数说明     hist / weight_sum / raw_count   输出直方图、权重总和、原始点计数
// 参数说明     line_min / line_max             可选，每行 x 极值（NULL 跳过）
// 返回参数     void
// 备注信息     权重 = y - y_start + 1（近景重远景轻）
//===================================================================================================================
static void collect_edge_hist(const int16 edge[][2], int edge_count,
                              int y_start, int y_end,
                              uint16 *hist, int *weight_sum, int *raw_count,
                              int16 *line_min, int16 *line_max)
{
    int y_span = y_end - y_start;
    if (line_min)
    {
        for (int i = 0; i < y_span; i++) line_min[i] = 32767;
    }
    if (line_max)
    {
        for (int i = 0; i < y_span; i++) line_max[i] = -32768;
    }
    for (int i = 0; i < edge_count; i++)
    {
        int y = edge[i][1];
        if (y >= y_start && y < y_end)
        {
            int x = edge[i][0];
            if (x >= 0 && x < WARP_IMAGE_W)
            {
                int w = y - y_start + 1;
                hist[x] = (uint16)(hist[x] + w);
                *weight_sum += w;
                (*raw_count)++;
                int idx = y - y_start;
                if (line_min && x < line_min[idx]) line_min[idx] = x;
                if (line_max && x > line_max[idx]) line_max[idx] = x;
            }
        }
    }
}

//===================================================================================================================
// 函数简介     计算图像偏差 error_image（PID 输入）
// 参数说明     void
// 返回参数     void（结果写入全局 error_image）
// 备注信息     采样区 [y=65,75)；直道取 median，弯道取 mode + curve_offset 内偏
//             单边降级用对侧 ± 赛道宽反推；异常时 error_image *= 0.7 衰减
//===================================================================================================================
void calc_error_image(void)
{
    // 前方采样区域
    int y_start = 50;
    int y_end = 60;
    int y_span = y_end - y_start;     // 10

    // 弯道偏移量（循边线时，目标位置相对图像中心的偏移）
    int16 curve_offset = 0;

    // 计数直方图（按 y 加权累计），权重 weight = y - y_start + 1，近景高远景低
    uint16 hist[WARP_IMAGE_W] = {0};
    uint16 hist_left[WARP_IMAGE_W] = {0};
    uint16 hist_right[WARP_IMAGE_W] = {0};
    int weight_sum = 0;
    int left_weight_sum = 0;
    int right_weight_sum = 0;
    int left_raw_count = 0;
    int right_raw_count = 0;

    // 单边降级时直接覆写虚拟中心（>=0 表示有效）
    int16 virtual_center_override = -1;
    // 主路径直方图算法选择：循单边时用 mode（抗三极管离群点），双边循中线时用 median
    int use_mode_for_main = 0;

    // 按 road_type 扫描边线 + 合成 hist + 单边 fallback
    uint8 at_t_junction = (current_junction == JUNCTION_LEFT_T  ||
                           current_junction == JUNCTION_RIGHT_T ||
                           current_junction == JUNCTION_T       ||
                           current_junction == JUNCTION_CROSS);
    int16 width_full = (road_width_avg > 0) ? (int16)road_width_avg : 60;
    int16 width_half = width_full / 2;

    switch (road_type)
    {
        case straight:
        {
            // 直道：双侧边线 + line_min/max 供交叉检测
            int16 left_min[10];
            int16 right_max[10];
            collect_edge_hist(left_edge,  left_edge_count,  y_start, y_end,
                              hist_left,  &left_weight_sum,  &left_raw_count,
                              left_min, NULL);
            collect_edge_hist(right_edge, right_edge_count, y_start, y_end,
                              hist_right, &right_weight_sum, &right_raw_count,
                              NULL, right_max);

            // 双边合并取 median ≈ 赛道中线
            for (int x = 0; x < WARP_IMAGE_W; x++)
            {
                hist[x] = (uint16)(hist_left[x] + hist_right[x]);
            }
            weight_sum = left_weight_sum + right_weight_sum;

            // 左右交叉 → 衰减返回
            for (int i = 0; i < y_span; i++)
            {
                if (left_min[i] != 32767 && right_max[i] != -32768 && left_min[i] >= right_max[i])
                {
                    error_image = error_image * 7 / 10;
                    return;
                }
            }
            // 两侧不足 → 衰减
            if (left_raw_count < 2 && right_raw_count < 2)
            {
                error_image = error_image * 7 / 10;
                return;
            }
            // 单边 fallback：用对侧主峰 ± 半宽反推虚拟中线（非 T 字）
            if (left_raw_count < 2 && right_raw_count >= 2 && !at_t_junction)
            {
                int16 right_peak = get_hist_mode_center(hist_right, right_weight_sum);
                virtual_center_override = right_peak - width_half;
            }
            else if (right_raw_count < 2 && left_raw_count >= 2 && !at_t_junction)
            {
                int16 left_peak = get_hist_mode_center(hist_left, left_weight_sum);
                virtual_center_override = left_peak + width_half;
            }
            break;
        }

        case left:
            // 左弯：curve_offset = +3 内偏
            curve_offset = 3;
            collect_edge_hist(left_edge, left_edge_count, y_start, y_end,
                              hist_left, &left_weight_sum, &left_raw_count,
                              NULL, NULL);
            if (left_raw_count >= 2)
            {
                // 主路径：左边线 mode 作为虚拟峰位
                for (int x = 0; x < WARP_IMAGE_W; x++)
                {
                    hist[x] = hist_left[x];
                }
                weight_sum = left_weight_sum;
                use_mode_for_main = 1;
            }
            else
            {
                // 左线不足 → 右侧 fallback
                collect_edge_hist(right_edge, right_edge_count, y_start, y_end,
                                  hist_right, &right_weight_sum, &right_raw_count,
                                  NULL, NULL);
                if (right_raw_count >= 2)
                {
                    int16 right_peak = get_hist_mode_center(hist_right, right_weight_sum);
                    virtual_center_override = right_peak - width_full;
                }
            }
            // 左右都不足时 virtual_center_override 仍为 -1 → 步骤2 衰减返回
            break;

        case right:
            // 右弯：curve_offset = -3 内偏
            curve_offset = -3;
            collect_edge_hist(right_edge, right_edge_count, y_start, y_end,
                              hist_right, &right_weight_sum, &right_raw_count,
                              NULL, NULL);
            if (right_raw_count >= 2)
            {
                // 主路径：右边线 mode 作为虚拟峰位
                for (int x = 0; x < WARP_IMAGE_W; x++)
                {
                    hist[x] = hist_right[x];
                }
                weight_sum = right_weight_sum;
                use_mode_for_main = 1;
            }
            else
            {
                // 右线不足 → 左侧 fallback
                collect_edge_hist(left_edge, left_edge_count, y_start, y_end,
                                  hist_left, &left_weight_sum, &left_raw_count,
                                  NULL, NULL);
                if (left_raw_count >= 2)
                {
                    int16 left_peak = get_hist_mode_center(hist_left, left_weight_sum);
                    virtual_center_override = left_peak + width_full;
                }
            }
            break;
    }

    // hist / virtual_center_override → virtual_center → error_image
    int16 virtual_center;
    if (virtual_center_override >= 0)
    {
        virtual_center = virtual_center_override;
    }
    else
    {
        // 至少需要 5 个原始点，否则误差向 0 衰减
        if (left_raw_count + right_raw_count < 5)
        {
            error_image = error_image * 7 / 10;
            return;
        }
        if (use_mode_for_main)
            virtual_center = get_hist_mode_center(hist, weight_sum);
        else
            virtual_center = get_hist_median(hist, weight_sum);
    }
    // clamp 到有效范围
    if (virtual_center < 0) virtual_center = 0;
    if (virtual_center >= WARP_IMAGE_W) virtual_center = WARP_IMAGE_W - 1;

    // 计算偏差
    int16 new_error = virtual_center - WARP_IMAGE_W / 2 - curve_offset;

    // 限幅
    if (new_error > 50) new_error = 50;
    if (new_error < -50) new_error = -50;

    // 平滑滤波
    error_image = 0.9f * new_error + 0.1f * last_error;
    last_error = error_image;
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
        return;
    }

    // dir_advance 事件响应：重置 PID
    if (dir_advance_pending)
    {
        dir_advance_count++;
        control_pid_reset();
        dir_advance_pending = 0;
    }


    // 计算图像偏差
    calc_error_image();


    // PID输出
    image_pid_out();


    // 电机控制
    motor_control(control_base_speed + output, control_base_speed - output);

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












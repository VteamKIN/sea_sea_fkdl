/*
 * imu.c
 *
 *      Author: aaa
 */


//imu660ra_acc_x, imu660ra_acc_y, imu660ra_acc_z
//imu660ra_gyro_x, imu660ra_gyro_y, imu660ra_gyro_z

#include "zf_common_headfile.h"

// ============================================================
// 将IMU相关变量放入 CPU0 数据RAM（IMU在中断中访问，控制模块在CPU0）
// ============================================================
#pragma section all "cpu0_dsram"

float imu_acc_x_f = 0, imu_acc_y_f = 0, imu_acc_z_f = 0;
float imu_gyro_x_f = 0, imu_gyro_y_f = 0, imu_gyro_z_f = 0;
float angle_out = 0.0f;  // 角度环输出值
float acc_y_offset = 0.0f;                  // Y轴加速度计静止零偏 (g)，校准后替代硬编码 1.0

// 换算角度
#define DT 0.002f

// 陀螺仪刻度补偿：标定方法 — 原地转90°，看 yaw_angle 实际显示值
#define GYRO_SCALE  (90.0f / 81.9f)

#define MODE_LINE 0
#define MODE_TURN 1
#define ranges_of_angle_error 7.0f
#define imu_base_speed 700


float yaw_angle = 0.0f;
float gyro_offset = 0.0f;           //陀螺仪零偏(校准后的均值)

//零飘死区阈值(deg/s)，低于此值的角速度视为噪声忽略
//典型IMU660RB在±125dps量程下静止噪声约±0.3deg/s，设0.5可滤除大部分零飘
#define GYRO_DEAD_ZONE 0.5f

int16 left_speed_imu;
int16 right_speed_imu;
// 转弯控制
uint8_t  run_mode = MODE_LINE;
float    target_angle = 0.0f;


/*
// 差速转弯 PID
float kp = 1.2f;
float kd = 0.4f;
float error, last_error;
float turn_pwm;
*/




/*****角度环*****/

// 角度环参数
float angle_kp = 0.0f;           // 比例系数
float angle_kd = 0.0f;           // 微分系数
float angle_output = 0.0f;       // 角度环输出
float angle_error = 0.0f;        // 角度误差
float angle_last_error = 0.0f;   // 上次角度误差
float angle_output_max = 500.0f; // 输出限幅

// 角度环状态
uint8_t angle_loop_enable = 0;   // 角度环使能标志

#pragma section all restore

//周期化限幅
float normalize_angle(float angle)
{
    while (angle > 180)  angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
}


//加速度计 Y 轴零偏校准（上电静止时调用）
void acc_calib(void)
{
    #define ACC_CALIB_SAMPLES 200
    float sum = 0;
    int i;
    for (i = 0; i < ACC_CALIB_SAMPLES; i++)
    {
        imu660rb_get_acc();
        sum += imu660rb_acc_transition(imu660rb_acc_y);
        system_delay_ms(2);
    }
    acc_y_offset = sum / (float)ACC_CALIB_SAMPLES;
}

//上电静止校准（去极值+均值，比纯均值更抗干扰）
void gyro_calib(void)
{
    //采样缓冲区
    #define CALIB_SAMPLES 500
    static float buf[CALIB_SAMPLES];
    float sum;
    int i, j;
    float temp;

    //采集原始数据
    for(i = 0; i < CALIB_SAMPLES; i++)
    {
        imu660rb_get_gyro();
        buf[i] = imu660rb_gyro_transition(imu660rb_gyro_y);
        system_delay_ms(1);
    }

    //冒泡排序
    for(i = 0; i < CALIB_SAMPLES - 1; i++)
    {
        for(j = 0; j < CALIB_SAMPLES - 1 - i; j++)
        {
            if(buf[j] > buf[j+1])
            {
                temp = buf[j];
                buf[j] = buf[j+1];
                buf[j+1] = temp;
            }
        }
    }

    //去掉最大最小各10%，取中间80%的均值
    int trim = CALIB_SAMPLES / 10;
    sum = 0;
    int count = 0;
    for(i = trim; i < CALIB_SAMPLES - trim; i++)
    {
        sum += buf[i];
        count++;
    }
    gyro_offset = sum / (float)count;
}




//换算角度（含零飘死区+刻度补偿）
void gyro_update(void)
{
    float wz = imu_gyro_x_f - gyro_offset;

    //零飘死区：角速度绝对值小于阈值视为噪声，置零
    if(fabs(wz) < GYRO_DEAD_ZONE)
        wz = 0.0f;

    //刻度补偿
    wz *= GYRO_SCALE;

    yaw_angle += wz * DT;
    yaw_angle = normalize_angle(yaw_angle);
    
}

/**
 * @brief 角度环初始化
 * @param kp 比例系数
 * @param kd 微分系数
 */
void angle_loop_init(float kp, float kd)
{
    angle_kp = kp;
    angle_kd = kd;
    angle_error = 0.0f;
    angle_last_error = 0.0f;
    angle_output = 0.0f;
    angle_loop_enable = 1;
}

/**
 * @brief 设置目标角度
 * @param target 目标角度(度)
 */
void angle_set_target(float target)
{
    target_angle = normalize_angle(target);
}

/**
 * @brief 角度环PD控制计算
 * @return 角度环输出值(用于差速控制)
 */
float angle_loop_update(void)
{
    if(!angle_loop_enable)
        return 0.0f;

    // 计算角度误差
    angle_error = normalize_angle(target_angle - yaw_angle);

    // PD控制器
    // P项: 比例控制，误差越大输出越大
    // D项: 微分控制，抑制角速度，增加阻尼
    angle_output = angle_kp * angle_error - angle_kd * imu_gyro_z_f;

    // 输出限幅
    if(angle_output > angle_output_max)
        angle_output = angle_output_max;
    else if(angle_output < -angle_output_max)
        angle_output = -angle_output_max;

    // 保存上次误差
    angle_last_error = angle_error;

    return angle_output;
}

/**
 * @brief 使能角度环
 */
void angle_loop_enable_set(uint8_t enable)
{
    angle_loop_enable = enable;
}

/**
 * @brief 重置角度环状态
 */
void angle_loop_reset(void)
{
    angle_error = 0.0f;
    angle_last_error = 0.0f;
    angle_output = 0.0f;
    yaw_angle = 0.0f;
    target_angle = 0.0f;
}

/**
 * @brief 相对角度转弯(在当前角度基础上转指定角度)
 * @param delta_angle 相对转动角度(正值为左转，负值为右转)
 */
void angle_turn_relative(float delta_angle)
{
    target_angle = normalize_angle(yaw_angle + delta_angle);
    angle_loop_enable = 1;
}

/**
 * @brief 绝对角度转弯(转到指定的绝对角度)
 * @param absolute_angle 目标绝对角度
 */
void angle_turn_absolute(float absolute_angle)
{
    target_angle = normalize_angle(absolute_angle);
    angle_loop_enable = 1;
}

/**
 * @brief 判断是否到达目标角度
 * @param threshold 允许的角度误差阈值(度)
 * @return 1=已到达, 0=未到达
 */
uint8_t angle_is_reached(float threshold)
{
    float error = fabs(normalize_angle(target_angle - yaw_angle));
    return (error < threshold) ? 1 : 0;
}

/**
 * @brief  IMU速度控制计算与电机输出
 * @note   在 isr.c 中断中调用，实时控制转弯差速
 */
void imu_control(void)
{
    left_speed_imu = imu_base_speed - (int16)angle_out;
    if(left_speed_imu > SPEED_MAX) left_speed_imu = SPEED_MAX;
    if(left_speed_imu < SPEED_MIN) left_speed_imu = SPEED_MIN;

    right_speed_imu = imu_base_speed + (int16)angle_out;
    if(right_speed_imu > SPEED_MAX) right_speed_imu = SPEED_MAX;
    if(right_speed_imu < SPEED_MIN) right_speed_imu = SPEED_MIN;

    motor_control(left_speed_imu, right_speed_imu);
}


/**
 * @brief  在当前角度基础上左转90°
 * @note   DEPRECATED: IMU 当前未启用（init.c 中 my_imu_init 被注释）。
 *         调用前请先启用 IMU，否则将等待 IMU_TURN_TIMEOUT_MS 后超时返回。
 */
void turn_left_90(void)
{
    //设置相对转弯角度（自动使能角度环）
    angle_turn_relative(90.0f);  // 左转90°

    //等待转弯完成（在主循环中轮询），带超时保护
    uint32 t0_ms = system_getval_ms();
    while (!angle_is_reached(ranges_of_angle_error))
    {
        // 角度环在中断中自动运行
        // 电机自动差速控制
        if ((system_getval_ms() - t0_ms) > IMU_TURN_TIMEOUT_MS) break;
    }

    //转弯完成，禁用角度环
    angle_loop_enable_set(0);
}

/**
 * @brief  在当前角度基础上右转90°
 * @note   DEPRECATED: IMU 当前未启用。超时保护同 turn_left_90。
 */
void turn_right_90(void)
{
    angle_turn_relative(-90.0f);

    uint32 t0_ms = system_getval_ms();
    while (!angle_is_reached(ranges_of_angle_error))
    {
        if ((system_getval_ms() - t0_ms) > IMU_TURN_TIMEOUT_MS) break;
    }

    angle_loop_enable_set(0);
}

/**
 * @brief  转到绝对角 90°
 * @note   DEPRECATED: IMU 当前未启用。超时保护同 turn_left_90。
 */
void turn_left_absolute_90(void)
{
    //设置绝对角度（自动使能角度环）
    angle_turn_absolute(0.0f);  // 转到90°

    //等待转弯完成（在主循环中轮询），带超时保护
    uint32 t0_ms = system_getval_ms();
    while (!angle_is_reached(ranges_of_angle_error))
    {
        // 角度环在中断中自动运行
        // 电机自动差速控制
        if ((system_getval_ms() - t0_ms) > IMU_TURN_TIMEOUT_MS) break;
    }

    //转弯完成，禁用角度环
    //angle_loop_enable_set(0);
}

/**
 * @brief  转到绝对角 -90°
 * @note   DEPRECATED: IMU 当前未启用。超时保护同 turn_left_90。
 */
void turn_right_absolute_90(void)
{
    angle_turn_absolute(-90.0f);

    uint32 t0_ms = system_getval_ms();
    while (!angle_is_reached(ranges_of_angle_error))
    {
        if ((system_getval_ms() - t0_ms) > IMU_TURN_TIMEOUT_MS) break;
    }

    angle_loop_enable_set(0);
}

/**
 * @brief  非阻塞式转弯函数 - 启动左转90°
 * @note   设置目标角度并使能角度环，立即返回
 */
void start_turn_left_90(void)
{
    angle_turn_relative(90.0f);  // 设置目标角度并使能角度环
}

/**
 * @brief  非阻塞式转弯函数 - 启动右转90°
 * @note   设置目标角度并使能角度环，立即返回
 */
void start_turn_right_90(void)
{
    angle_turn_relative(-90.0f);
}

/**
 * @brief  检查转弯是否完成
 * @return 1=完成, 0=未完成
 */
uint8_t is_turn_complete(void)
{
    if (angle_is_reached(ranges_of_angle_error))
    {
        angle_loop_enable_set(0);  // 禁用角度环
        return 1;
    }
    return 0;
}

/**
 * @brief  额外的imu初始化
 * @note   调试距离环时禁用角度环
 */
void my_imu_init(void)
{
    // 角度环初始化（调试距离环时禁用）
    angle_loop_init(6.0, 0.1);
    
    // 陀螺仪校准（调试距离环时禁用）
    gyro_calib();

    // 加速度计 Y 轴零偏校准
    acc_calib();
    
    // 角度环定时中断（调试距离环时禁用）
    pit_ms_init(CCU60_CH1, 2);
    
    // 禁用角度环
    angle_loop_enable = 0;
}

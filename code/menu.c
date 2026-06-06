/*
 * menu.c
 * 参数组快速切换菜单
 * 仅在发车前使用：KEY1/KEY2 切换参数组，KEY3 发车并锁定菜单
 */

#include "zf_common_headfile.h"

#define MENU_FIXED_PROFILE_COUNT 6
#define MENU_PROFILE_COUNT       (1 + MENU_FIXED_PROFILE_COUNT)

/* 单个参数组：保存发车前可以安全切换的运行参数。 */
typedef struct
{
    const char *name;
    float kp_value;
    float ki_value;
    float kd_value;
    int16 base_speed;
    int16 lookahead;
    int16 error_limit;
    int16 inward_bias;
    int16 cross_pulses;
    int16 curve_exit_pulses;
    uint8 line_lost_enable;
    uint8 choose_len;
    int8 choose_values[road_num];
} menu_profile_t;

static uint8 current_profile_index = 0;
static menu_profile_t current_profile;
static uint8 current_profile_ready = 0;
static uint8 menu_locked_after_launch = 0;  // KEY3 发车后置 1，运行中菜单不再响应按键。

/* 固定参数组  索引 0 保留给 Current 当前参数。 */
static const menu_profile_t fixed_profiles[MENU_FIXED_PROFILE_COUNT] =
{
    {
        "1500 Qual",
        18.0f, 0.02f, 0.0f,
        1500, 85, 50, 3,
        3200, 1000,
        0,
        18,
        {0,1,1,1,1,1,1,0,1,0,0,0,1,1,0,0,0,-1}
    },
    {
        "2000 Qual",
        23.0f, 0.00f, 0.0f,
        2000, 120, 50, -3,
        2500, 1000,
        0,
        18,
        {0,1,1,1,1,1,1,0,1,0,0,0,1,1,0,0,0,-1}
    },
    {
        "2300 Qual",
        19.0f, 0.00f, 13.6f,
        2300, 120, 60, -3,
        2500, 1000,
        1,
        18,
        {0,1,1,1,1,1,1,0,1,0,0,0,1,1,0,0,0,-1}
    },
    {
        "2500 Qual",
        23.0f, 0.00f, 13.6f,
        2500, 120, 100, -3,
        2500, 1000,
        1,
        18,
        {0,1,1,1,1,1,1,0,1,0,0,0,1,1,0,0,0,-1}
    },
    {
        "1500 Final",
        18.0f, 0.02f, 0.0f,
        1500, 85, 50, 3,
        3200, 1000,
        0,
        30,
        {1,1,0,2,3,1,0,1,1,0,1,2,0,1,0,1,1,1,3,1,1,0,0,0,2,2,1,1,0,-1}
    },
    {
        "2300 Final",
        19.0f, 0.00f, 13.6f,
        2300, 120, 60, -3,
        3200, 1000,
        0,
        25,
        {1,1,0,2,3,1,0,1,1,0,1,2,0,1,0,1,1,1,3,1,1,0,0,0,2,2,1,1,0,-1}
    }
};

/* 根据屏幕索引取得参数组指针；0 表示启动时捕获的当前工程参数。 */
static const menu_profile_t *menu_get_profile(uint8 index)
{
    if (index == 0)
    {
        return &current_profile;
    }
    return &fixed_profiles[index - 1];
}

/* 只在启动时快照一次当前工程参数，后续可切回 Current。 */
static void menu_capture_current_profile(void)
{
    if (current_profile_ready)
    {
        return;
    }

    current_profile.name = "Current";
    current_profile.kp_value = kp;
    current_profile.ki_value = ki;
    current_profile.kd_value = kd;
    current_profile.base_speed = control_base_speed;
    current_profile.lookahead = pursuit_lookahead;
    current_profile.error_limit = error_limit;
    current_profile.inward_bias = pursuit_inward_bias;
    current_profile.cross_pulses = cross_ignore_pulses;
    current_profile.curve_exit_pulses = curve_exit_ignore_pulses;
    current_profile.line_lost_enable = line_lost_protect_enable;
    current_profile.choose_len = road_num;

    for (uint8 i = 0; i < road_num; i++)
    {
        current_profile.choose_values[i] = choose[i];
    }

    current_profile_ready = 1;
}

/* 切换参数组或发车前重置路径和控制状态。 */
static void menu_reset_road_state(void)
{
    dir_count = 0;
    dir_advance_pending = 0;
    dir_advance_count = 0;
    current_target_dir = choose[0];
    road_type = straight;
    current_junction = JUNCTION_NONE;
    raw_junction_debug = JUNCTION_NONE;
    junction_detected = 0;
    cross_active = 0;
    control_line_lost_reset();
    control_pid_reset();
}

/* 将选中的参数组写入控制和图像处理正在使用的全局变量。 */
static void menu_apply_profile(const menu_profile_t *profile)
{
    kp = profile->kp_value;
    ki = profile->ki_value;
    kd = profile->kd_value;
    control_base_speed = profile->base_speed;
    pursuit_lookahead = profile->lookahead;
    error_limit = profile->error_limit;
    pursuit_inward_bias = profile->inward_bias;
    cross_ignore_pulses = profile->cross_pulses;
    curve_exit_ignore_pulses = profile->curve_exit_pulses;
    line_lost_protect_enable = profile->line_lost_enable;

    // 未使用的路径槽位填 -1，避免残留上一个参数组的方向。
    for (uint8 i = 0; i < road_num; i++)
    {
        choose[i] = -1;
    }

    uint8 copy_len = profile->choose_len;
    if (copy_len > road_num)
    {
        copy_len = road_num;
    }

    for (uint8 i = 0; i < copy_len; i++)
    {
        choose[i] = profile->choose_values[i];
    }

    menu_reset_road_state();
    __dsync();
}

/* 按 8x16 字体行显示：y = 0,16,...,112，避免超过 128 像素高度。 */
static void menu_show_profile(void)
{
    const menu_profile_t *profile = menu_get_profile(current_profile_index);

    tft180_clear();
    tft180_show_string(5, 0, "PARAM SET");

    tft180_show_string(5, 16, "No:");
    tft180_show_int(30, 16, (int)current_profile_index, 1);
    tft180_show_string(50, 16, (char *)profile->name);

    tft180_show_string(5, 32, "Kp:");
    tft180_show_float(30, 32, kp, 3, 1);
    tft180_show_string(85, 32, "Ki:");
    tft180_show_float(110, 32, ki, 2, 2);

    tft180_show_string(5, 48, "Kd:");
    tft180_show_float(30, 48, kd, 3, 1);
    tft180_show_string(85, 48, "V:");
    tft180_show_int(105, 48, control_base_speed, 4);

    tft180_show_string(5, 64, "LA:");
    tft180_show_int(30, 64, pursuit_lookahead, 3);
    tft180_show_string(75, 64, "Lim:");
    tft180_show_int(110, 64, error_limit, 3);

    tft180_show_string(5, 80, "Lost:");
    tft180_show_string(50, 80, line_lost_protect_enable ? "ON" : "OFF");

    tft180_show_string(5, 112, "K1/K2 Switch K3 Go");
}
/* 初始化启动菜单，并强制保持发车前停车状态。 */
void menu_init(void)
{
    menu_capture_current_profile();
    current_profile_index = 0;
    menu_apply_profile(menu_get_profile(current_profile_index));
    car_running = 0;
    menu_locked_after_launch = 0;
    menu_show_profile();
}

/* 仅启动前有效的按键处理；KEY3 发车后锁定菜单。 */
void menu_process(void)
{
    if (menu_locked_after_launch)
    {
        key_clear_all_state();
        return;
    }

    if (!car_running && key_get_state(KEY_1) == KEY_SHORT_PRESS)
    {
        current_profile_index = (uint8)((current_profile_index + MENU_PROFILE_COUNT - 1) % MENU_PROFILE_COUNT);
        menu_apply_profile(menu_get_profile(current_profile_index));
        menu_show_profile();
    }

    if (!car_running && key_get_state(KEY_2) == KEY_SHORT_PRESS)
    {
        current_profile_index = (uint8)((current_profile_index + 1) % MENU_PROFILE_COUNT);
        menu_apply_profile(menu_get_profile(current_profile_index));
        menu_show_profile();
    }

    if (!car_running && key_get_state(KEY_3) == KEY_SHORT_PRESS)
    {
        menu_reset_road_state();
        car_running = 1;
        menu_locked_after_launch = 1;
        tft180_clear();
    }

    key_clear_all_state();
}

/* 兼容旧调试入口：旧代码可能仍会调用 menu_show_pid()。 */
void menu_show_pid(void)
{
    menu_show_profile();
}

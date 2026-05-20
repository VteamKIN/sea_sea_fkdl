/*
 * menu.c
 * PID参数调节菜单实现
 * 一套PID
 * Created on: 2026年3月14日
 *      Author: aaa
 */

#include "zf_common_headfile.h"

/*******************************************************************************
 * 菜单参数定义
 *******************************************************************************/

// 参数调节步长
#define KP_STEP  0.5f
#define KI_STEP  0.01f
#define KD_STEP  0.5f

// 主菜单枚举
typedef enum {
    MENU_MAIN_STATUS = 0,
    MENU_MAIN_PID,
    MENU_MAIN_CHOOSE,
    MENU_MAIN_COUNT
} menu_main_t;

// 菜单层级
typedef enum {
    MENU_PAGE_MAIN = 0,
    MENU_PAGE_STATUS,
    MENU_PAGE_PID,
    MENU_PAGE_CHOOSE,
    MENU_PAGE_COUNT
} menu_page_t;

// PID参数枚举
typedef enum {
    MENU_KP = 0,
    MENU_KI,
    MENU_KD,
    MENU_PARAM_COUNT
} menu_state_t;

// 菜单变量
static menu_main_t current_main = MENU_MAIN_STATUS;
static menu_page_t current_page = MENU_PAGE_MAIN;
static menu_state_t current_param = MENU_KP;
static uint8 choose_cursor = 0;        // CHOOSE 页面当前编辑位置 0~(road_num-1)

// 函数前向声明
static void menu_show_main_list(void);
static void menu_show_status(void);
static void menu_show_choose(void);
static void handle_main_key(void);
static void handle_status_key(void);
static void handle_pid_key(void);
static void handle_choose_key(void);
static float* get_current_param_ptr(void);
static float get_current_step(void);
void menu_show_pid(void);

typedef struct {
    void (*show)(void);
    void (*handle_key)(void);
} menu_page_handler_t;

static const menu_page_handler_t menu_pages[MENU_PAGE_COUNT] = {
    { menu_show_main_list, handle_main_key },
    { menu_show_status, handle_status_key },
    { menu_show_pid, handle_pid_key },
    { menu_show_choose, handle_choose_key }
};

/*******************************************************************************
 * 显示主菜单列表
 *******************************************************************************/
static void menu_show_main_list(void)
{
    tft180_clear();

    tft180_show_string(60, 5, "MENU");

    tft180_show_string(5, 25, "1.Status");
    tft180_show_string(90, 25, car_running ? "RUN" : "STOP");
    if (current_main == MENU_MAIN_STATUS)
    {
        tft180_show_string(140, 25, "<");
    }

    tft180_show_string(5, 50, "2.PID");
    tft180_show_string(90, 50, "Adjust");
    if (current_main == MENU_MAIN_PID)
    {
        tft180_show_string(140, 50, "<");
    }

    tft180_show_string(5, 75, "3.Choose");
    tft180_show_string(90, 75, "Path");
    if (current_main == MENU_MAIN_CHOOSE)
    {
        tft180_show_string(140, 75, "<");
    }

    tft180_show_string(5, 105, "K1/K2 Sel  K3 Enter");

}

/*******************************************************************************
 * 显示当前运行状态
 *******************************************************************************/
static void menu_show_status(void)
{
    tft180_clear();

    tft180_show_string(50, 5, "STATUS");

    tft180_show_string(5, 25, "Car:");
    tft180_show_string(40, 25, car_running ? "RUN" : "STOP");

    tft180_show_string(5, 45, "Kp:");
    tft180_show_float(30, 45, kp, 3, 1);

    tft180_show_string(5, 60, "Ki:");
    tft180_show_float(30, 60, ki, 3, 2);

    tft180_show_string(5, 75, "Kd:");
    tft180_show_float(30, 75, kd, 3, 1);

    tft180_show_string(5, 95, "K2 Start/Stop");
    tft180_show_string(5, 110, "K4 Back to Menu");

}

/*******************************************************************************
 * 显示PID参数
 *******************************************************************************/
void menu_show_pid(void)
{
    tft180_clear();

    tft180_show_string(5, 5, "--- PID ---");
    tft180_show_string(5, 20, "Kp:");
    tft180_show_float(35, 20, kp, 3, 1);
    if (current_param == MENU_KP)
    {
        tft180_show_string(90, 20, "<");
    }

    tft180_show_string(5, 35, "Ki:");
    tft180_show_float(35, 35, ki, 3, 2);
    if (current_param == MENU_KI)
    {
        tft180_show_string(90, 35, "<");
    }

    tft180_show_string(5, 50, "Kd:");
    tft180_show_float(35, 50, kd, 3, 1);
    if (current_param == MENU_KD)
    {
        tft180_show_string(90, 50, "<");
    }

    tft180_show_string(5, 80, "K1 Next Param");
    tft180_show_string(5, 95, "K2 Inc  K3 Dec");
    tft180_show_string(5, 110, "K4 Back to Menu");

}

/*******************************************************************************
 * 菜单初始化
 *******************************************************************************/
void menu_init(void)
{
    current_main = MENU_MAIN_STATUS;
    current_page = MENU_PAGE_MAIN;
    current_param = MENU_KP;

    menu_pages[current_page].show();
}

/*******************************************************************************
 * 获取当前选中参数的指针
 *******************************************************************************/
static float* get_current_param_ptr(void)
{
    switch (current_param)
    {
        case MENU_KP:
            return &kp;
        case MENU_KI:
            return &ki;
        case MENU_KD:
            return &kd;
        default:
            return &kp;
    }
}

//获取当前参数的步长
static float get_current_step(void)
{
    switch (current_param)
    {
        case MENU_KI:
            return KI_STEP;
        case MENU_KP:
            return KP_STEP;
        case MENU_KD:
            return KD_STEP;
        default:
            return KP_STEP;
    }
}

/*******************************************************************************
 * 主菜单按键处理
 *******************************************************************************/
static void handle_main_key(void)
{
    if (key_get_state(KEY_1) == KEY_SHORT_PRESS)
    {
        current_main = (menu_main_t)((current_main + MENU_MAIN_COUNT - 1) % MENU_MAIN_COUNT);
        menu_show_main_list();
    }

    if (key_get_state(KEY_2) == KEY_SHORT_PRESS)
    {
        current_main = (menu_main_t)((current_main + 1) % MENU_MAIN_COUNT);
        menu_show_main_list();
    }

    if (key_get_state(KEY_3) == KEY_SHORT_PRESS)
    {
        if (current_main == MENU_MAIN_STATUS)
        {
            current_page = MENU_PAGE_STATUS;
            menu_show_status();
        }
        else if (current_main == MENU_MAIN_PID)
        {
            current_page = MENU_PAGE_PID;
            menu_show_pid();
        }
        else
        {
            current_page = MENU_PAGE_CHOOSE;
            menu_show_choose();
        }
    }
}

/*******************************************************************************
 * 状态页面按键处理
 *******************************************************************************/
static void handle_status_key(void)
{
    if (key_get_state(KEY_2) == KEY_SHORT_PRESS)
    {
        car_running = !car_running;
        if (!car_running)
        {
            motor_control(0, 0);
        }
        menu_show_status();
    }

    if (key_get_state(KEY_4) == KEY_SHORT_PRESS)
    {
        current_page = MENU_PAGE_MAIN;
        menu_show_main_list();
    }
}

/*******************************************************************************
 * PID页面按键处理
 *******************************************************************************/
static void handle_pid_key(void)
{
    if (key_get_state(KEY_1) == KEY_SHORT_PRESS)
    {
        current_param = (menu_state_t)((current_param + 1) % MENU_PARAM_COUNT);
        menu_show_pid();
    }

    if (key_get_state(KEY_2) == KEY_SHORT_PRESS)
    {
        *get_current_param_ptr() += get_current_step();
        menu_show_pid();
    }

    if (key_get_state(KEY_3) == KEY_SHORT_PRESS)
    {
        *get_current_param_ptr() -= get_current_step();
        menu_show_pid();
    }

    if (key_get_state(KEY_4) == KEY_SHORT_PRESS)
    {
        current_page = MENU_PAGE_MAIN;
        menu_show_main_list();
    }
}

/*******************************************************************************
 * CHOOSE 页面：编辑 choose[] 路径数组
 * - 4x5 网格显示 20 个槽位，每个槽位 32 像素宽
 * - 当前光标位置用 [X] 标识，其它槽位为  X
 * - 值显示: L=循左(0) / R=循右(1) / S=停车(-1)
 *******************************************************************************/
static char choose_value_char(int8 v)
{
    if (v == 0)  return 'L';
    if (v == 1)  return 'R';
    if (v == -1) return 'S';
    return '?';
}

static void menu_show_choose(void)
{
    tft180_clear();

    tft180_show_string(5, 5, "-- CHOOSE PATH --");

    // 顶部状态行：当前光标位置 + 当前值
    tft180_show_string(5, 22, "Idx:");
    tft180_show_int(40, 22, (int)choose_cursor, 2);
    tft180_show_string(75, 22, "Val:");
    char vstr[2];
    vstr[0] = choose_value_char(choose[choose_cursor]);
    vstr[1] = '\0';
    tft180_show_string(110, 22, vstr);

    // 4x5 网格：每个 cell 32px 宽，16px 高
    char cell[4];
    for (int row = 0; row < 4; row++)
    {
        for (int col = 0; col < 5; col++)
        {
            int idx = row * 5 + col;
            uint16 x = (uint16)(col * 32);
            uint16 y = (uint16)(43 + row * 16);
            char c = choose_value_char(choose[idx]);
            if (idx == choose_cursor)
            {
                cell[0] = '[';
                cell[1] = c;
                cell[2] = ']';
            }
            else
            {
                cell[0] = ' ';
                cell[1] = c;
                cell[2] = ' ';
            }
            cell[3] = '\0';
            tft180_show_string(x, y, cell);
        }
    }

    tft180_show_string(5, 112, "K1Nx K2+ K3- K4Bk");
}

/*******************************************************************************
 * CHOOSE 页面按键处理
 * K1 光标后移   K2 值循环+   K3 值循环-   K4 返回
 *******************************************************************************/
static void handle_choose_key(void)
{
    if (key_get_state(KEY_1) == KEY_SHORT_PRESS)
    {
        choose_cursor = (uint8)((choose_cursor + 1) % road_num);
        menu_show_choose();
    }

    if (key_get_state(KEY_2) == KEY_SHORT_PRESS)
    {
        // 值循环: -1 -> 0 -> 1 -> -1
        int8 v = choose[choose_cursor];
        if      (v == -1) v = 0;
        else if (v ==  0) v = 1;
        else              v = -1;
        choose[choose_cursor] = v;
        __dsync();    // 跨核可见性: choose[] 位于 cpu1_dsram
        menu_show_choose();
    }

    if (key_get_state(KEY_3) == KEY_SHORT_PRESS)
    {
        // 值反向循环: -1 -> 1 -> 0 -> -1
        int8 v = choose[choose_cursor];
        if      (v == -1) v = 1;
        else if (v ==  1) v = 0;
        else              v = -1;
        choose[choose_cursor] = v;
        __dsync();
        menu_show_choose();
    }

    if (key_get_state(KEY_4) == KEY_SHORT_PRESS)
    {
        current_page = MENU_PAGE_MAIN;
        menu_show_main_list();
    }
}

/*******************************************************************************
 * 菜单处理函数
 *******************************************************************************/
void menu_process(void)
{
    if (current_page < MENU_PAGE_COUNT)
    {
        if (menu_pages[current_page].handle_key != NULL)
        {
            menu_pages[current_page].handle_key();
        }
    }
    // 清除所有按键状态，防止 menu_process 在主循环高频调用时
    // 同一次 KEY_SHORT_PRESS 被重复响应（key_scanner 仅在按键释放时设置一次该状态）
    key_clear_all_state();
}

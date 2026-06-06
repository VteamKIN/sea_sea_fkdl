/*
 * param_tune.c   上位机调参模块
 *
 * 详细说明见 param_tune.h
 */
#include "zf_common_headfile.h"
#include "param_tune.h"


#pragma section all "cpu0_dsram"

// ============================================================
// 参数表：在这里登记所有可调参数
// 类型：PT_FLOAT / PT_U16 / PT_I16 / PT_U16_ARR
// 名称匹配大小写敏感，长度 <= 15
// ============================================================
typedef enum
{
    PT_FLOAT,    // float
    PT_U8,       // uint8
    PT_U16,      // uint16
    PT_I16,      // int16
    PT_U16_ARR,  // uint16 数组元素（addr 指向数组首地址，extra=索引）
} param_type_t;

typedef struct
{
    const char  *name;
    void        *addr;
    param_type_t type;
    uint16       extra;   // 数组索引（仅 PT_U16_ARR 用）
} param_entry_t;

// 参数表
static const param_entry_t g_params[] =
{
    // PID 参数
    { "kp",  &kp,  PT_FLOAT, 0 },
    { "ki",  &ki,  PT_FLOAT, 0 },
    { "kd",  &kd,  PT_FLOAT, 0 },
    // 十字路口穿越参数
    { "cross", &cross_ignore_pulses,  PT_I16, 0 },   // 十字直行忽略窗口(编码器脉冲)
    { "cv_exit", &curve_exit_ignore_pulses, PT_I16, 0 }, // 弯道退弯后忽略窗口
};

#define PARAM_COUNT  (sizeof(g_params) / sizeof(g_params[0]))
#define LINE_BUF_MAX 64

// 行缓冲（跨调用累积，遇到换行才解析）
static char  g_line_buf[LINE_BUF_MAX];
static uint8 g_line_idx = 0;

#pragma section all restore

// ============================================================
// 字符串处理工具
// ============================================================
static int str_eq(const char *a, const char *b)
{
    while (*a && *b)
    {
        if (*a != *b) return 0;
        a++; b++;
    }
    return (*a == 0 && *b == 0);
}

// 解析 float
static float parse_float(const char *s)
{
    int   sign  = 1;
    float intp  = 0.0f;
    float fracp = 0.0f;
    float div   = 1.0f;

    if (*s == '-') { sign = -1; s++; }
    else if (*s == '+') { s++; }

    while (*s >= '0' && *s <= '9')
    {
        intp = intp * 10.0f + (float)(*s - '0');
        s++;
    }
    if (*s == '.')
    {
        s++;
        while (*s >= '0' && *s <= '9')
        {
            fracp = fracp * 10.0f + (float)(*s - '0');
            div  *= 10.0f;
            s++;
        }
    }
    return (float)sign * (intp + fracp / div);
}

// 解析整数（支持负号）
static int32 parse_int(const char *s)
{
    int   sign = 1;
    int32 v    = 0;
    if (*s == '-') { sign = -1; s++; }
    else if (*s == '+') { s++; }
    while (*s >= '0' && *s <= '9')
    {
        v = v * 10 + (*s - '0');
        s++;
    }
    return sign * v;
}

// ============================================================
// 命令处理
// ============================================================
static void print_param(const param_entry_t *p)
{
    switch (p->type)
    {
        case PT_FLOAT:
            printf("  %s = %.4f\n", p->name, *(float *)p->addr);
            break;
        case PT_U8:
            printf("  %s = %u\n", p->name, (unsigned)(*(uint8 *)p->addr));
            break;
        case PT_U16:
            printf("  %s = %u\n", p->name, (unsigned)(*(uint16 *)p->addr));
            break;
        case PT_I16:
            printf("  %s = %d\n", p->name, (int)(*(int16 *)p->addr));
            break;
        case PT_U16_ARR:
            printf("  %s = %u\n", p->name, (unsigned)(((uint16 *)p->addr)[p->extra]));
            break;
    }
}

static void list_all_params(void)
{
    printf("---- params ----\n");
    for (uint32 i = 0; i < PARAM_COUNT; i++)
        print_param(&g_params[i]);
    printf("----------------\n");
}

static void handle_set(char *name, char *value)
{
    for (uint32 i = 0; i < PARAM_COUNT; i++)
    {
        if (!str_eq(name, g_params[i].name)) continue;

        const param_entry_t *p = &g_params[i];
        switch (p->type)
        {
            case PT_FLOAT:
                *(float *)p->addr = parse_float(value);
                break;
            case PT_U8:
                *(uint8 *)p->addr = (uint8)parse_int(value);
                break;
            case PT_U16:
                *(uint16 *)p->addr = (uint16)parse_int(value);
                break;
            case PT_I16:
                *(int16 *)p->addr = (int16)parse_int(value);
                break;
            case PT_U16_ARR:
                ((uint16 *)p->addr)[p->extra] = (uint16)parse_int(value);
                break;
        }
        printf("OK: ");
        print_param(p);
        return;
    }
    printf("ERR: unknown param '%s'\n", name);
}

// 处理一行命令（line 已以 '\0' 结尾，无换行符）
static void handle_line(char *line)
{
    if (line[0] == 0) return;

    // 查询命令
    if (line[0] == '?' && line[1] == 0)
    {
        list_all_params();
        return;
    }

    // 找 '='
    char *eq = line;
    while (*eq && *eq != '=') eq++;
    if (*eq != '=')
    {
        printf("ERR: expect 'name=value', got '%s'\n", line);
        return;
    }
    *eq = 0;
    handle_set(line, eq + 1);
}

// ============================================================
// 主循环入口（非阻塞）
// ============================================================
void param_tune_process(void)
{
    uint8 rx[32];
    uint32 n = wireless_uart_read_buffer(rx, sizeof(rx));
    if (n == 0) return;

    for (uint32 i = 0; i < n; i++)
    {
        char c = (char)rx[i];

        if (c == '\n' || c == '\r')
        {
            // 行结束 → 解析
            g_line_buf[g_line_idx] = 0;
            handle_line(g_line_buf);
            g_line_idx = 0;
        }
        else if (c == 0x08 || c == 0x7F)   // 退格
        {
            if (g_line_idx > 0) g_line_idx--;
        }
        else if (g_line_idx < LINE_BUF_MAX - 1)
        {
            g_line_buf[g_line_idx++] = c;
        }
        else
        {
            // 行太长：丢弃
            g_line_idx = 0;
            printf("ERR: line too long\n");
        }
    }
}

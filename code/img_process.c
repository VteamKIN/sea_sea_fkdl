/*
 * img_process.c
 * 阿巴实现
 * 主要功能：我是奶龙
 * Created on: 2026年3月31日
 *      Author: aaa
 */

#include "zf_common_headfile.h"
#include "isr.h"

// ============================================================
// 将图像处理相关变量放入 CPU1 数据RAM）
// ============================================================
#pragma section all "cpu1_dsram"

// 图像处理计时（微秒）
vuint16 img_process_time = 0;         // 图像处理耗时(μs)

// 方向增量表：上、右、下、左
static const int dir_front[4][2]      = {{0,-1},{1,0},{0,1},{-1,0}};
// 左前方向增量表：用于左手边线跟踪
static const int dir_frontleft[4][2]  = {{-1,-1},{1,-1},{1,1},{-1,1}};
// 右前方向增量表：用于右手边线跟踪
static const int dir_frontright[4][2] = {{1,-1},{1,1},{-1,1},{-1,-1}};


int16 left_edge[MAX_EDGE_POINTS][2];         // 左边线坐标 [i][0]=x [i][1]=y
vuint8 left_edge_count = 0;          // 左边线点数
int16 right_edge[MAX_EDGE_POINTS][2];        // 右边线坐标 [i][0]=x [i][1]=y
vuint8 right_edge_count = 0;         // 右边线点数


vuint8  dir_count = 0;               // 当前弯道序号(0~19)
static uint8 last_junction_detected = 0;  // 上一帧 junction_detected（用于检测上升沿）
int8  choose[road_num] = {0,1,-1}; // 方向选择 0:循左线(yaw) 1:循右线(yaw) 2:直行穿越 3:循左线(编码器) 4:循右线(编码器) -1:停止
int16 cross_ignore_pulses = CROSS_IGNORE_PULSES_DEFAULT; // 十字直行忽略窗口（编码器脉冲累计）
int16 curve_exit_ignore_pulses = CURVE_EXIT_IGNORE_PULSES_DEFAULT; // 弯道退弯后忽略窗口（编码器脉冲累计）
vint8 current_target_dir = 0;        // 当前循线方向(choose[dir_count]的缓存)
vuint8 dir_advance_pending = 0;      // dir_count 推进事件信号（detect_road_type 退弯瞬间设 1，control.c 处理后清 0）
vuint8 cross_active = 0;             // 1=cross straight ignore window active

volatile RUN_Dir road_type =straight;       // 道路类型 straight/left/right
volatile enum JunctionType current_junction = JUNCTION_NONE;  // 当前路口类型
vuint8  road_width_avg = 0;          // 平均道路宽度(像素)
vuint8  junction_detected = 0;       // 路口检测标志 0:无 1:有
volatile enum JunctionType raw_junction_debug = JUNCTION_NONE; // 路口检测原始值（供调试显示）
volatile uint8 left_slope_mutation = 0;      // 左边线是否检测到斜率突变（方案E输出，调试可见）
volatile uint8 right_slope_mutation = 0;     // 右边线是否检测到斜率突变（方案E输出，调试可见）
vuint8 element_repair_active = 0;             // 元器件补线状态
vuint8 element_repair_bad_rows = 0;           // 本帧异常/缺失行数
vuint8 element_repair_fixed_rows = 0;         // 本帧补线行数
vuint8 element_repair_cross_rows = 0;         // 本帧左右边线相交/过近行数
static enum JunctionType last_raw_junction = JUNCTION_NONE;   // 上一次原始路口检测结果(防抖用)
static uint8 junction_stable_count = 0;      // 路口检测防抖计数
static uint8 edge_start_lost = 0;            // 边线起始点丢失标志

static float  Mat1[3][3]=       { { 0.462294745070467, -0.405415828307522, 42.7673376252083},
        { -0.0686056632504893, 0.704497456356808, 5.69914538551316},
        { -0.000503555802327882, -0.00475326158864043, 0.856296641858558}, };
static float Tx = 0;                         // 逆透视变换中间结果x
static float Ty = 0;                         // 逆透视变换中间结果y

uint8 warp_image[WARP_IMAGE_H][WARP_IMAGE_W];  // 逆透视图像缓冲
uint8 *warp_image_ptr = &warp_image[0][0];   // 逆透视图像指针(供TFT/Send使用)

uint16 row_prefix[WARP_IMAGE_H][WARP_IMAGE_W + 1];   // 行前缀和数组

volatile uint8 global_warp_thres = 0;        // 底部最终阈值

static uint16 warp_lut[WARP_IMAGE_H][WARP_IMAGE_W];  // 逆透视LUT: 输出像素→源图偏移


#pragma section all restore
//===================================================================================================================
// 函数简介
// 参数说明
// 返回参数
// 使用示例
// 备注信息
//===================================================================================================================


//===================================================================================================================
// 函数简介     计算单行平均阈值
// 参数说明     y       计算的y坐标
// 参数说明     width   计算的宽度（一般为图像宽度）
// 返回参数     uint8
// 使用示例     calc_row_mean_thres(y, IMG_H)
// 备注信息     计算出的阈值会减小ROAD_THRES_CLIP
//===================================================================================================================
static uint8 calc_row_mean_thres(int y, int width)
{
    uint32 sum = row_prefix[y][width];
    uint8 thres = (uint8)(sum / (uint32)width);
    if (thres > ROAD_THRES_CLIP)
        thres = (uint8)(thres - ROAD_THRES_CLIP);
    return thres;
}

//===================================================================================================================
// 函数简介     差比和黑白跳变检测
// 参数说明     p1 p2      两个点（相邻）
// 返回参数     1   存在跳变    0  不存在跳变
// 使用示例     cbh_is_jump(p1, p2)
// 备注信息
//===================================================================================================================
static uint8 cbh_is_jump(uint8 p1, uint8 p2)
{
    uint16 sum = (uint16)p1 + (uint16)p2;
    if (sum == 0)
        return 0;

    int16 diff = (int16)p1 - (int16)p2;
    int16 ratio = (int16)((diff << 7) / (int16)sum);

    if (ratio > CBH_RATIO_THRESHOLD)
        return 1;
    else
        return 0;
}


//===================================================================================================================
static int element_abs_int(int v)
{
    return (v < 0) ? -v : v;
}

static int16 element_clamp_x(int value)
{
    if (value < 0) return 0;
    if (value > WARP_IMAGE_W - 1) return WARP_IMAGE_W - 1;
    return (int16)value;
}

static void element_build_row_edges(int16 left_x[], int16 right_x[])
{
    for (int y = 0; y < WARP_IMAGE_H; y++)
    {
        left_x[y] = -1;
        right_x[y] = -1;
    }

    for (int i = 0; i < left_edge_count; i++)
    {
        int y = left_edge[i][1];
        int x = left_edge[i][0];
        if (y >= 0 && y < WARP_IMAGE_H && x >= 0 && x < WARP_IMAGE_W)
        {
            if (left_x[y] < 0 || x < left_x[y])
                left_x[y] = (int16)x;
        }
    }

    for (int i = 0; i < right_edge_count; i++)
    {
        int y = right_edge[i][1];
        int x = right_edge[i][0];
        if (y >= 0 && y < WARP_IMAGE_H && x >= 0 && x < WARP_IMAGE_W)
        {
            if (right_x[y] < 0 || x > right_x[y])
                right_x[y] = (int16)x;
        }
    }
}

static void element_rebuild_edge_points(const int16 out_left[], const int16 out_right[],
                                        const uint8 valid[])
{
    int left_num = 0;
    int right_num = 0;

    for (int y = WARP_IMAGE_H - 1; y >= 0; y--)
    {
        if (!valid[y])
            continue;

        if (left_num < MAX_EDGE_POINTS)
        {
            left_edge[left_num][0] = out_left[y];
            left_edge[left_num][1] = (int16)y;
            left_num++;
        }

        if (right_num < MAX_EDGE_POINTS)
        {
            right_edge[right_num][0] = out_right[y];
            right_edge[right_num][1] = (int16)y;
            right_num++;
        }
    }

    left_edge_count = (uint8)left_num;
    right_edge_count = (uint8)right_num;
}

//===================================================================================================================
// 函数简介     元器件道路中断修复：按 y 行重建边线并补 bad/missing 段
// 参数说明     void
// 返回参数     void
// 备注信息     只在直道非十字窗口启用；补线点只供本帧控制/路口检测使用，模型仅由稳定帧更新
//===================================================================================================================
static void repair_element_break_lines(void)
{
#if ELEMENT_REPAIR_ENABLE
    static int16 model_center[WARP_IMAGE_H];
    static uint8 model_valid[WARP_IMAGE_H];
    static int16 model_width = ELEMENT_REPAIR_DEFAULT_WIDTH;
    static uint8 model_ready = 0;
    static uint8 repair_active = 0;
    static uint8 recover_frames = 0;

    int16 left_x[WARP_IMAGE_H];
    int16 right_x[WARP_IMAGE_H];
    int16 center[WARP_IMAGE_H];
    int16 fill_center[WARP_IMAGE_H];
    int16 out_left[WARP_IMAGE_H];
    int16 out_right[WARP_IMAGE_H];
    uint8 valid[WARP_IMAGE_H];
    uint8 fill_valid[WARP_IMAGE_H];
    uint8 repaired[WARP_IMAGE_H];

    element_repair_bad_rows = 0;
    element_repair_fixed_rows = 0;
    element_repair_cross_rows = 0;

    if (cross_active || road_type != straight)
    {
        repair_active = 0;
        recover_frames = 0;
        element_repair_active = 0;
        return;
    }

    element_build_row_edges(left_x, right_x);

    int width_sum = 0;
    int width_count = 0;
    for (int y = 0; y < WARP_IMAGE_H; y++)
    {
        if (left_x[y] >= 0 && right_x[y] >= 0)
        {
            int width = right_x[y] - left_x[y];
            if (width >= ELEMENT_REPAIR_MIN_WIDTH && width <= ELEMENT_REPAIR_MAX_WIDTH)
            {
                width_sum += width;
                width_count++;
            }
        }
    }

    int16 width_used;
    if (width_count >= 4)
        width_used = (int16)(width_sum / width_count);
    else if (road_width_avg >= ELEMENT_REPAIR_MIN_WIDTH)
        width_used = (int16)road_width_avg;
    else if (model_ready)
        width_used = model_width;
    else
        width_used = ELEMENT_REPAIR_DEFAULT_WIDTH;

    if (width_used < ELEMENT_REPAIR_MIN_WIDTH)
        width_used = ELEMENT_REPAIR_MIN_WIDTH;
    if (width_used > ELEMENT_REPAIR_MAX_WIDTH)
        width_used = ELEMENT_REPAIR_MAX_WIDTH;

    int width_tol = width_used / 2;
    if (width_tol < ELEMENT_REPAIR_WIDTH_TOL)
        width_tol = ELEMENT_REPAIR_WIDTH_TOL;
    int width_min = width_used - width_tol;
    int width_max = width_used + width_tol;
    if (width_min < ELEMENT_REPAIR_MIN_WIDTH)
        width_min = ELEMENT_REPAIR_MIN_WIDTH;

    int valid_rows = 0;
    int first_valid = -1;
    int last_valid = -1;
    int bottom_valid_rows = 0;

    for (int y = 0; y < WARP_IMAGE_H; y++)
    {
        center[y] = -1;
        fill_center[y] = -1;
        out_left[y] = -1;
        out_right[y] = -1;
        valid[y] = 0;
        fill_valid[y] = 0;
        repaired[y] = 0;

        if (left_x[y] >= 0 && right_x[y] >= 0)
        {
            int width = right_x[y] - left_x[y];
            if (width <= ELEMENT_REPAIR_CROSS_MIN_GAP)
            {
                element_repair_cross_rows++;
                element_repair_bad_rows++;
                continue;
            }
            if (width < width_min || width > width_max)
            {
                element_repair_bad_rows++;
                continue;
            }

            center[y] = (int16)((left_x[y] + right_x[y]) / 2);
            valid[y] = 1;
        }
    }

    int prev_valid = 0;
    int prev_center = 0;
    for (int y = WARP_IMAGE_H - 1; y >= 0; y--)
    {
        if (!valid[y])
            continue;

        if (prev_valid && element_abs_int((int)center[y] - prev_center) > ELEMENT_REPAIR_CENTER_JUMP)
        {
            valid[y] = 0;
            element_repair_bad_rows++;
            continue;
        }

        prev_valid = 1;
        prev_center = center[y];
    }

    if (model_ready)
    {
        for (int y = 0; y < WARP_IMAGE_H; y++)
        {
            if (valid[y] && model_valid[y] &&
                element_abs_int((int)center[y] - (int)model_center[y]) > ELEMENT_REPAIR_CENTER_JUMP)
            {
                valid[y] = 0;
                element_repair_bad_rows++;
            }
        }
    }

    for (int y = 0; y < WARP_IMAGE_H; y++)
    {
        if (!valid[y])
            continue;

        valid_rows++;
        if (first_valid < 0)
            first_valid = y;
        last_valid = y;
        if (y >= WARP_IMAGE_H - BOTTOM_SAMPLE_ROWS)
            bottom_valid_rows++;
    }

    uint8 repair_needed = 0;
    if (element_repair_bad_rows > 0 || element_repair_cross_rows > 0)
        repair_needed = 1;
    if (model_ready && (edge_start_lost || valid_rows < ELEMENT_REPAIR_MODEL_MIN_ROWS))
        repair_needed = 1;
    if (model_ready && bottom_valid_rows < 2 && valid_rows > 0)
        repair_needed = 1;

    // 先保留当前帧可信行，再补中间断段。
    for (int y = 0; y < WARP_IMAGE_H; y++)
    {
        if (valid[y])
        {
            fill_center[y] = center[y];
            fill_valid[y] = 1;
        }
    }

    if (valid_rows >= 2)
    {
        int y = first_valid;
        while (y <= last_valid)
        {
            if (fill_valid[y])
            {
                y++;
                continue;
            }

            int start = y;
            while (y <= last_valid && !fill_valid[y])
                y++;
            int end = y - 1;
            int above = start - 1;
            int below = end + 1;
            int len = end - start + 1;

            if (above >= 0 && below < WARP_IMAGE_H && fill_valid[above] && fill_valid[below] &&
                len <= ELEMENT_REPAIR_MAX_GAP_ROWS)
            {
                int dy = below - above;
                int dc = fill_center[below] - fill_center[above];
                for (int yy = start; yy <= end; yy++)
                {
                    fill_center[yy] = (int16)(fill_center[above] + dc * (yy - above) / dy);
                    fill_valid[yy] = 1;
                    repaired[yy] = 1;
                    element_repair_fixed_rows++;
                }
            }
        }
    }

    // 异常状态下，底部/顶部缺锚点时必须用进入前模型或最近可信行延伸补出边线。
    if (repair_needed)
    {
        for (int y = 0; y < WARP_IMAGE_H; y++)
        {
            if (fill_valid[y])
                continue;

            if (model_ready && model_valid[y])
            {
                fill_center[y] = model_center[y];
                fill_valid[y] = 1;
                repaired[y] = 1;
                element_repair_fixed_rows++;
            }
        }

        // 如果模型还没覆盖某些行，用同帧最近可信行向外延伸，避免底部电容/中空区域无点可用。
        int nearest_y = -1;
        for (int y = 0; y < WARP_IMAGE_H; y++)
        {
            if (fill_valid[y])
            {
                nearest_y = y;
            }
            else if (nearest_y >= 0)
            {
                fill_center[y] = fill_center[nearest_y];
                fill_valid[y] = 1;
                repaired[y] = 1;
                element_repair_fixed_rows++;
            }
        }

        nearest_y = -1;
        for (int y = WARP_IMAGE_H - 1; y >= 0; y--)
        {
            if (fill_valid[y])
            {
                nearest_y = y;
            }
            else if (nearest_y >= 0)
            {
                fill_center[y] = fill_center[nearest_y];
                fill_valid[y] = 1;
                repaired[y] = 1;
                element_repair_fixed_rows++;
            }
        }
    }

    int fill_rows = 0;
    for (int y = 0; y < WARP_IMAGE_H; y++)
    {
        if (!fill_valid[y])
            continue;

        fill_rows++;
        if (!repaired[y] && left_x[y] >= 0 && right_x[y] >= 0)
        {
            out_left[y] = left_x[y];
            out_right[y] = right_x[y];
        }
        else
        {
            int left = fill_center[y] - width_used / 2;
            int right = left + width_used;
            out_left[y] = element_clamp_x(left);
            out_right[y] = element_clamp_x(right);
        }
    }

    if ((repair_needed || element_repair_fixed_rows > 0) &&
        fill_rows >= ELEMENT_REPAIR_MODEL_MIN_ROWS)
    {
        element_rebuild_edge_points(out_left, out_right, fill_valid);
        repair_active = 1;
        recover_frames = 0;
    }
    else
    {
        if (repair_active)
        {
            if (recover_frames < 255)
                recover_frames++;
            if (recover_frames >= ELEMENT_REPAIR_RECOVER_FRAMES)
                repair_active = 0;
        }
    }

    if (width_count >= 4)
    {
        if (road_width_avg < ELEMENT_REPAIR_MIN_WIDTH)
            road_width_avg = (uint8)width_used;
        else
            road_width_avg = (uint8)(((int)road_width_avg * 3 + width_used) / 4);
    }

    // 模型只从当前帧真实可信行学习，不从补线行学习。
    if (!repair_active && valid_rows >= ELEMENT_REPAIR_MODEL_MIN_ROWS)
    {
        for (int y = 0; y < WARP_IMAGE_H; y++)
        {
            if (valid[y])
            {
                model_center[y] = center[y];
                model_valid[y] = 1;
            }
            else
            {
                model_valid[y] = 0;
            }
        }
        model_width = width_used;
        model_ready = 1;
    }

    element_repair_active = repair_active;
#endif
}

// 函数简介     构建积分图 计算单行像素和
// 参数说明     *img    图像指针
// 返回参数     void
// 使用示例     build_row_prefix(warp_image_ptr)
// 备注信息     具体原理：integral[y][x] = Σ(0≤dy<y, 0≤dx<x) img[dy][dx]
// 备注信息     区域和算法：sum(x1,y1,x2,y2) = integral[y2][x2] - integral[y1][x2] - integral[y2][x1] + integral[y1][x1]
//===================================================================================================================
void build_row_prefix(image_t *img)

{
    const int w = img->width;
    const int h = img->height;
    const uint8 *data = img->data;

    for (int y = 0; y < h; y++)
    {
        row_prefix[y][0] = 0;
        const uint8 *row = data + y * w;
        for (int x = 0; x < w; x++)
        {
            row_prefix[y][x + 1] = row_prefix[y][x] + row[x];
        }
    }
}

//===================================================================================================================
// 函数简介     底部起点检测 更新底部阈值
// 参数说明     *img_data  img_w  img_h                         图像数据结构
// 返回参数     *out_road_left  *out_road_right  *out_y_start   返回值的指针
// 使用示例     find_start_point_warp(*img, IMG_W, IMG_H, road_left, road_right, y_start)
// 备注信息     扫描底部 BOTTOM_SAMPLE_ROWS 行
// 备注信息     限幅跟随防抖：每帧最多偏移 ±8px；连续丢失 20 帧后重置到图像中心
//===================================================================================================================
static void find_start_point_warp(const uint8 *img_data, int img_w, int img_h,
                                  int *out_road_left, int *out_road_right, int *out_y_start)
{
    int y_start = img_h - 1;

    int sample_rows = BOTTOM_SAMPLE_ROWS;
    if (sample_rows > img_h)
        sample_rows = img_h;
    int sample_start_y = img_h - sample_rows;

    // 底部各行最值中点阈值（光照鲁棒：暗/亮场景都能定位赛道）
    // 阈值 = (row_min + row_max) / 2 - clip；对比度不足的行标记为无效，不参与亮段搜索

    uint8 row_thres_buf[BOTTOM_SAMPLE_ROWS];    // 底部阈值存储数组
    uint8 row_valid_buf[BOTTOM_SAMPLE_ROWS];    // 标记有效数组
    uint32 thres_sum = 0;
    uint8 last_valid_t = 128;
    for (int i = 0; i < sample_rows; i++)
    {
        const uint8 *row = img_data + (sample_start_y + i) * img_w;
        uint8 row_min = 255, row_max = 0;
        // 获取单行最大/小灰度值
        for (int x = 0; x < img_w; x++)
        {
            uint8 v = row[x];
            if (v < row_min) row_min = v;
            if (v > row_max) row_max = v;
        }
        uint8 t;
        if ((int)row_max - (int)row_min < MIN_ROAD_CONTRAST)
        {
            // 对比度不足（暗均匀地面或无赛道）：阈值用上次有效值，标记无效
            t = last_valid_t;
            row_valid_buf[i] = 0;
        }
        else
        {
            t = (uint8)(((uint16)row_min + (uint16)row_max) / 2);
            if (t > ROAD_THRES_CLIP) t = (uint8)(t - ROAD_THRES_CLIP);
            last_valid_t = t;
            row_valid_buf[i] = 1;
        }
        row_thres_buf[i] = t;
        thres_sum += t;
    }
    global_warp_thres = (uint8)(thres_sum / sample_rows);

    // 跨帧道路状态
    static int last_center  = -1;
    static int last_left_s  = -1;
    static int last_right_s = -1;
    static int lost_frames  = 0;

    if (last_center < 0) last_center = img_w / 2;//上帧无中心的默认值

    // 配对候选：每行找离上帧中心最近的合法亮段，左右保持同行配对
    int16 pair_left[BOTTOM_SAMPLE_ROWS];
    int16 pair_right[BOTTOM_SAMPLE_ROWS];
    int16 pair_y[BOTTOM_SAMPLE_ROWS];            //记录每个配对对应的y坐标
    int pair_count = 0;

    for (int i = 0; i < sample_rows; i++)
    {
        if (!row_valid_buf[i]) continue;  // 对比度不足的行跳过（避免均匀地面被误切出亮段）
        const uint8 *row = img_data + (sample_start_y + i) * img_w;// 单行对应列的像素值
        uint8 row_thres = row_thres_buf[i];

        int b_start = -1, b_len = 0, b_dist = img_w;

        // 间隙跨越扫描：允许小暗间隙（元器件中空）不打断区间
        int gap_max = (road_width_avg > 10) ? (int)(road_width_avg * 3 / 4) : ELEMENT_GAP_MAX_DEFAULT;
        int rs = -1;    // 区间起始 x
        int re = -1;    // 区间最后一个白像素 x
        int gap = 0;    // 当前连续暗像素计数

        for (int x = 0; x < img_w; x++)
        {
            if (row[x] > row_thres)
            {
                if (rs == -1) rs = x;
                re = x;
                gap = 0;
            }
            else if (rs != -1)
            {
                gap++;
                if (gap > gap_max)
                {
                    // 间隙过大，结算当前区间
                    int span = re - rs + 1;
                    if (span >= NARROW_WIDTH_MIN)
                    {
                        int rc = (rs + re) / 2;
                        int d = rc - last_center;
                        if (d < 0) d = -d;
                        if (d < b_dist) { b_dist = d; b_start = rs; b_len = span; }
                    }
                    rs = -1; re = -1; gap = 0;
                }
            }
        }
        // 处理行尾区间
        if (rs != -1)
        {
            int span = re - rs + 1;
            if (span >= NARROW_WIDTH_MIN)
            {
                int rc = (rs + re) / 2;
                int d = rc - last_center;
                if (d < 0) d = -d;
                if (d < b_dist) { b_start = rs; b_len = span; }
            }
        }

        if (b_start != -1)
        {
            pair_left[pair_count]  = (int16)b_start;
            pair_right[pair_count] = (int16)(b_start + b_len - 1);
            pair_y[pair_count]    = (int16)(sample_start_y + i);
            pair_count++;
        }
    }

    int best_left = -1;
    int best_right = -1;
    int best_y = img_h - 1;

    if (pair_count > 0)
    {
        // 按道路中心冒泡排序（≤8 个），取中位数对
        for (int i = 0; i < pair_count - 1; i++)
        {
            for (int j = i + 1; j < pair_count; j++)
            {
                int ci = pair_left[i] + pair_right[i];
                int cj = pair_left[j] + pair_right[j];
                if (ci > cj)
                {
                    int16 tmp;
                    tmp = pair_left[i];  pair_left[i]  = pair_left[j];  pair_left[j]  = tmp;
                    tmp = pair_right[i]; pair_right[i] = pair_right[j]; pair_right[j] = tmp;
                    tmp = pair_y[i];     pair_y[i]     = pair_y[j];     pair_y[j]     = tmp;
                }
            }
        }
        int mid = pair_count / 2;
        best_left  = pair_left[mid];
        best_right = pair_right[mid];
        best_y     = pair_y[mid];

    }

    // 底部全失败时向上再搜 16 行兜底
    if (best_left == -1 || best_right == -1)
    {
        int search_start = sample_start_y - 16;
        if (search_start < 0) search_start = 0;
        for (int y = sample_start_y - 1; y >= search_start; y--)
        {
            const uint8 *row = img_data + y * img_w;
            uint8 row_min = 255, row_max = 0;
            for (int x = 0; x < img_w; x++)
            {
                uint8 v = row[x];
                if (v < row_min) row_min = v;
                if (v > row_max) row_max = v;
            }
            // 对比度不足跳过此行
            if ((int)row_max - (int)row_min < MIN_ROAD_CONTRAST) continue;
            uint8 t = (uint8)(((uint16)row_min + (uint16)row_max) / 2);
            if (t > ROAD_THRES_CLIP) t = (uint8)(t - ROAD_THRES_CLIP);

            int b_start = -1, b_len = 0;
            int fb_gap_max = (road_width_avg > 10) ? (int)(road_width_avg * 3 / 4) : ELEMENT_GAP_MAX_DEFAULT;
            int rs = -1, re = -1, fb_gap = 0;
            for (int x = 0; x < img_w; x++)
            {
                if (row[x] > t)
                {
                    if (rs == -1) rs = x;
                    re = x;
                    fb_gap = 0;
                }
                else if (rs != -1)
                {
                    fb_gap++;
                    if (fb_gap > fb_gap_max)
                    {
                        int span = re - rs + 1;
                        if (span > b_len) { b_len = span; b_start = rs; }
                        rs = -1; re = -1; fb_gap = 0;
                    }
                }
            }
            if (rs != -1)
            {
                int span = re - rs + 1;
                if (span > b_len) { b_len = span; b_start = rs; }
            }
            if (b_len >= NARROW_WIDTH_MIN)
            {
                best_left  = b_start;
                best_right = b_start + b_len - 1;
                y_start = y;
                break;
            }
        }
    }

    // 限幅跟随防抖（窄赛道平滑容易偏出道路） 若超出则使用上次的值 + max_shift
    const int max_shift = 8;//最大偏移量
    if (best_left == -1 || best_right == -1 || (best_right - best_left + 1) < NARROW_WIDTH_MIN)
    {
        edge_start_lost = 1;
        lost_frames++;
        if (lost_frames > 20)//有边丢线或不满足道路宽度
        {
            last_left_s  = img_w / 2 - 3;//默认值
            last_right_s = img_w / 2 + 3;
            last_center  = img_w / 2;
        }
        best_left  = (last_left_s >= 0) ? last_left_s : img_w / 2 - 3;//取上次的值
        best_right = (last_right_s >= 0) ? last_right_s : img_w / 2 + 3;
    }
    else
    {
        edge_start_lost = 0;
        lost_frames = 0;
        if (last_left_s < 0)
        {
            last_left_s  = best_left;
            last_right_s = best_right;
        }
        else
        {
            int nl = best_left, nr = best_right;
            if (nl - last_left_s >  max_shift) nl = last_left_s + max_shift;
            if (last_left_s - nl >  max_shift) nl = last_left_s - max_shift;
            if (nr - last_right_s > max_shift) nr = last_right_s + max_shift;
            if (last_right_s - nr > max_shift) nr = last_right_s - max_shift;
            last_left_s  = nl;
            last_right_s = nr;
            best_left  = nl;
            best_right = nr;
        }
        last_center = (best_left + best_right) / 2;
    }

    //输出
    *out_road_left  = best_left;
    *out_road_right = best_right;
    //y_start：正常路径=best_y(中位数行)，兜底路径已在上方赋值，丢失路径=img_h-1
    if (pair_count > 0)
        y_start = best_y;
    *out_y_start    = y_start;
}

//===================================================================================================================
// 函数简介     自适应边线爬取（基于左右手跟踪）
// 参数说明     void
// 返回参数     void
// 使用示例     zi_shi_ying_warp()
// 备注信息
//===================================================================================================================
void zi_shi_ying_warp(void)
{
    image_t img_raw = DEF_IMAGE(NULL, WARP_IMAGE_W, WARP_IMAGE_H);
    img_raw.data = &warp_image[0][0];

    build_row_prefix(&img_raw);


    // 重置边线点数
    left_edge_count = 0;
    right_edge_count = 0;

    // ===找到底部道路位置，确定跟踪起点（独立函数）===
    int road_left, road_right, y_start;
    find_start_point_warp(img_raw.data, img_raw.width, img_raw.height,
                          &road_left, &road_right, &y_start);




    // 调用左右手跟踪
    // 左边线用左手跟踪（贴着左边界走），右边线用右手跟踪（贴着右边界走）
    // 起点在边界上，确保7×7窗口内有黑有白，自适应阈值才能正常工作
    int half = ZSY_BLOCK_SIZE / 2;
    int start_left_x = (road_left > half) ? road_left : half + 1;
    int start_right_x = (road_right < img_raw.width - half - 1) ? road_right : img_raw.width - half - 2;
    int start_y = (y_start < img_raw.height - half) ? y_start : img_raw.height - half - 1;


    // 左边线：使用左手跟踪
    int16 left_pts_num = MAX_EDGE_POINTS;
    findline_lefthand_adaptive(&img_raw, ZSY_BLOCK_SIZE, ZSY_CLIP_VALUE,
                                start_left_x, start_y, left_edge, &left_pts_num);
    left_edge_count = (uint8)left_pts_num;

    // 右边线：使用右手跟踪
    int16 right_pts_num = MAX_EDGE_POINTS;
    findline_righthand_adaptive(&img_raw, ZSY_BLOCK_SIZE, ZSY_CLIP_VALUE,
                                 start_right_x, start_y, right_edge, &right_pts_num);
    right_edge_count = (uint8)right_pts_num;


    // ===边线回折截断===
    // 电容等元器件的横线会导致跟踪器折返，边线出现连续y递增（向下走）
    // 正常边线y递减（向上走），弯道处最多1-2个点y微增（转折噪声）
    // 连续y递增超过阈值则视为回折，截断后续点
    int fold_count = 0;
    for (int i = 1; i < left_edge_count; i++)
    {
        if (left_edge[i][1] > left_edge[i - 1][1])
            fold_count++;
        else
            fold_count = 0;
        if (fold_count >= EDGE_FOLD_THRESHOLD)
        {
            left_edge_count = i - fold_count + 1;
            break;
        }
    }
    fold_count = 0;
    for (int i = 1; i < right_edge_count; i++)
    {
        if (right_edge[i][1] > right_edge[i - 1][1])
            fold_count++;
        else
            fold_count = 0;
        if (fold_count >= EDGE_FOLD_THRESHOLD)
        {
            right_edge_count = i - fold_count + 1;
            break;
        }
    }

    // 元器件导致左右边线相交/中断时，在图像层按 y 行补回直道边线。
    repair_element_break_lines();

}

//===================================================================================================================
// 函数简介     左手边线跟踪
// 参数说明     *img        图像指针
// 参数说明     block_size  检测窗口
// 参数说明     clip_value  自适应阈值偏移量
// 参数说明     x y         跟踪起始点
// 参数说明     pts[][2]    存储坐标数组
// 参数说明     *num        跟踪步数限制+实际步数存入
// 返回参数     void
// 使用示例     findline_lefthand_adaptive(&img_raw, ZSY_BLOCK_SIZE, ZSY_CLIP_VALUE, start_left_x, start_y, left_edge, &left_pts_num);
// 备注信息     循内侧爬线（看左前方） 适用于窄赛道
//===================================================================================================================
void findline_lefthand_adaptive(image_t *img, int block_size, int clip_value, int x, int y, int16 pts[][2], int16 *num)
{
    int half = block_size / 2;
    int step = 0, dir = 0, turn = 0;
    const int img_w = img->width;
    const uint8 *img_data = img->data;

    while (step < *num && x >= half && x < img_w - half &&
           y >= half && y < img->height - half && turn < 4)
    {
        int x1 = x - half, y1 = y - half;
        int x2 = x + half + 1, y2 = y + half + 1;
        uint32 area_sum = 0;
        for (int yy = y1; yy < y2; yy++)
            area_sum += row_prefix[yy][x2] - row_prefix[yy][x1];
        int local_thres = (int)(area_sum / (block_size * block_size)) + clip_value;


        int front_value      = img_data[(y + dir_front[dir][1]) * img_w + (x + dir_front[dir][0])];
        int frontleft_value  = img_data[(y + dir_frontleft[dir][1]) * img_w + (x + dir_frontleft[dir][0])];

        // 判断前方是否为黑色（边界）
        if (front_value < local_thres)
        {
            // 右转
            dir = (dir + 1) % 4;
            turn++;
        }
        // 判断左前方是否为黑色
        else if (frontleft_value < local_thres)
        {
            // 前进一格
            x += dir_front[dir][0];
            y += dir_front[dir][1];
            pts[step][0] = (int16)x;
            pts[step][1] = (int16)y;
            step++;
            turn = 0;
        }
        else
        {
            // 左前方为白色，沿左前方前进并左转
            x += dir_frontleft[dir][0];
            y += dir_frontleft[dir][1];
            dir = (dir + 3) % 4;
            pts[step][0] = (int16)x;
            pts[step][1] = (int16)y;
            step++;
            turn = 0;
        }
    }
    // 返回实际步数
    *num = (int16)step;
}

//===================================================================================================================
// 函数简介     右手边线跟踪
// 参数说明     *img        图像指针
// 参数说明     block_size  检测窗口
// 参数说明     clip_value  自适应阈值偏移量
// 参数说明     x y         跟踪起始点
// 参数说明     pts[][2]    存储坐标数组
// 参数说明     *num        跟踪步数限制+实际步数存入
// 返回参数     void
// 使用示例     findline_righthand_adaptive(&img_raw, ZSY_BLOCK_SIZE, ZSY_CLIP_VALUE, start_right_x, start_y, right_edge, &right_pts_num);
// 备注信息     循内侧爬线（看右前方） 适用于窄赛道
//===================================================================================================================
void findline_righthand_adaptive(image_t *img, int block_size, int clip_value, int x, int y, int16 pts[][2], int16 *num)
{
    int half = block_size / 2;
    int step = 0, dir = 0, turn = 0;
    const int img_w = img->width;
    const uint8 *img_data = img->data;

    while (step < *num && x >= half && x < img_w - half &&
           y >= half && y < img->height - half && turn < 4)
    {
        int x1 = x - half, y1 = y - half;
        int x2 = x + half + 1, y2 = y + half + 1;
        uint32 area_sum = 0;
        for (int yy = y1; yy < y2; yy++)
            area_sum += row_prefix[yy][x2] - row_prefix[yy][x1];
        int local_thres = (int)(area_sum / (block_size * block_size)) + clip_value;

        int front_value       = img_data[(y + dir_front[dir][1]) * img_w + (x + dir_front[dir][0])];

        int frontright_value  = img_data[(y + dir_frontright[dir][1]) * img_w + (x + dir_frontright[dir][0])];

        // 判断前方是否为黑色（边界）
        if (front_value < local_thres)
        {
            // 左转
            dir = (dir + 3) % 4;
            turn++;
        }
        // 判断右前方是否为黑色
        else if (frontright_value < local_thres)
        {
            // 前进一格
            x += dir_front[dir][0];
            y += dir_front[dir][1];
            pts[step][0] = (int16)x;
            pts[step][1] = (int16)y;
            step++;
            turn = 0;
        }
        else
        {
            // 右前方为白色，沿右前方前进并右转
            x += dir_frontright[dir][0];
            y += dir_frontright[dir][1];
            dir = (dir + 1) % 4;
            pts[step][0] = (int16)x;
            pts[step][1] = (int16)y;
            step++;
            turn = 0;
        }
    }
    // 返回实际步数
    *num = (int16)step;
}


//===================================================================================================================
// 函数简介     边线斜率突变检测
// 参数说明     void
// 返回参数     void
// 使用示例     detect_edge_slope_mutation()
// 备注信息     计算近段和远段之间的叉乘 再*100与SLOPE_CHANGE_THRESHOLD进行比较
// 备注信息     叉乘：|far_dx*near_dy - near_dx*far_dy| / (far_dy*near_dy)
//===================================================================================================================
static uint8 detect_single_edge_slope_mutation(int16 edge[][2], uint8 edge_count)
{
    if (edge_count < EDGE_MIN_POINTS)
    {
        return 0;
    }

    for (int base = 0; base + EDGE_SLOPE_FAR_IDX_HI < edge_count; base += 2)
    {
        int near_lo = base + EDGE_SLOPE_NEAR_IDX_LO;
        int near_hi = base + EDGE_SLOPE_NEAR_IDX_HI;
        int far_lo  = base + EDGE_SLOPE_FAR_IDX_LO;
        int far_hi  = base + EDGE_SLOPE_FAR_IDX_HI;

        int16 near_dx = edge[near_hi][0] - edge[near_lo][0];
        int16 near_dy = edge[near_lo][1] - edge[near_hi][1];
        int16 far_dx  = edge[far_hi][0]  - edge[far_lo][0];
        int16 far_dy  = edge[far_lo][1]  - edge[far_hi][1];

        if (near_dy > 0 && far_dy > 0)
        {
            int32 cross = (int32)far_dx * near_dy - (int32)near_dx * far_dy;
            int32 abs_cross = cross < 0 ? -cross : cross;
            int32 norm = (int32)far_dy * near_dy;
            int16 slope_change = (int16)((abs_cross * 100) / norm);
            if (slope_change >= SLOPE_CHANGE_THRESHOLD)
                return 1;
        }
    }

    return 0;
}

void detect_edge_slope_mutation(void)
{
    left_slope_mutation = 0;
    right_slope_mutation = 0;

    left_slope_mutation = detect_single_edge_slope_mutation(left_edge, left_edge_count);
    right_slope_mutation = detect_single_edge_slope_mutation(right_edge, right_edge_count);
}

//===================================================================================================================
// 函数简介     路口检测（边线端点到达图像边界）
// 参数说明     void
// 返回参数     void
// 使用示例     detect_junction_type()
// 备注信息     检测左/右边线最后一个点的x坐标是否到达图像左右边界 → 弯道/路口
//===================================================================================================================
void detect_junction_type(void)
{
    // 丢线保持：起始点丢失时维持上一帧检测结果
    if (edge_start_lost)
        return;

    // 斜率突变检测（区分真弯道与倾斜直道）
    detect_edge_slope_mutation();

    // 检测最后一个边线点的x坐标是否到达图像边界 → 弯道
    uint8 curve_detected = 0;

    if (left_edge_count > 0)
    {
        int16 x = left_edge[left_edge_count - 1][0];
        if (x <= 2 || x >= WARP_IMAGE_W - 3)
            curve_detected = 1;
    }

    if (!curve_detected && right_edge_count > 0)
    {
        int16 x = right_edge[right_edge_count - 1][0];
        if (x <= 2 || x >= WARP_IMAGE_W - 3)
            curve_detected = 1;
    }

    // 边线到达边界 且 有斜率突变 → 真弯道；否则可能是倾斜直道
    if (curve_detected && !(left_slope_mutation || right_slope_mutation))
        curve_detected = 0;

    enum JunctionType raw_junction = curve_detected ? JUNCTION_CURVE : JUNCTION_NONE;
    raw_junction_debug = raw_junction;

    // 防抖：连续 N 帧一致才确认
    {
        uint8 raw_active  = (raw_junction  != JUNCTION_NONE) ? 1 : 0;
        uint8 last_active = (last_raw_junction != JUNCTION_NONE) ? 1 : 0;

        if (raw_active == last_active)
        {
            if (junction_stable_count < 255) junction_stable_count++;
            if (junction_stable_count >= JUNCTION_STABLE_THRESHOLD)
            {
                current_junction = raw_junction;
                junction_detected = raw_active;
            }
        }
        else
        {
            junction_stable_count = 0;
        }
        last_raw_junction = raw_junction;
    }
}

//===================================================================================================================
// 函数简介     道路方向综合判断（三态状态机）
// 参数说明     void
// 返回参数     void
// 使用示例     detect_road_type()
// 备注信息     STATE_STRAIGHT: 等待 junction 上升沿
//              STATE_CURVE:    循边线，yaw 或编码器退弯
//              STATE_CROSS:    十字直行穿越，编码器忽略窗口
//===================================================================================================================
void detect_road_type(void)
{
    // 边界检查
    if (dir_count >= road_num)
    {
        road_type = straight;
        cross_active = 0;
        last_junction_detected = junction_detected;
        return;
    }

    int8 target = choose[dir_count];
    current_target_dir = target;

    // 停车标志：road_type=straight，control.c 自行处理停车
    if (target == -1)
    {
        road_type = straight;
        cross_active = 0;
        last_junction_detected = junction_detected;
        return;
    }

    // 检测 junction_detected 上升沿（0→1）
    uint8 junction_rising = (junction_detected && !last_junction_detected) ? 1 : 0;
    last_junction_detected = junction_detected;

    // 三态状态机
    static uint8     state                 = 0;        // 0=STRAIGHT, 1=CURVE, 2=CROSS
    static RUN_Dir   latched_dir           = straight; // CURVE 状态下循的边线
    static uint8     latched_encoder_exit  = 0;        // 1=编码器阈值退弯，0=yaw 退弯
    static float     curve_entry_yaw       = 0.0f;     // 进弯时的 yaw 角
    static uint16    curve_frame_count     = 0;        // CURVE 状态持续帧数
    static uint8     yaw_complete_count    = 0;
    static int32     encoder_exit_start    = 0;        // 编码器退弯/忽略窗口的全局累计起点
    static uint8     post_curve_ignore     = 0;        // 退弯后忽略同一路口残留检测
    static int32     post_curve_ignore_start = 0;

    switch (state)
    {
    case 0: // ===== STATE_STRAIGHT：循中线，等待 junction 上升沿 =====
        road_type = straight;
        if (post_curve_ignore)
        {
            if (curve_exit_ignore_pulses <= 0)
            {
                post_curve_ignore = 0;
            }
            else
            {
                int32 ignore_delta = cross_encoder_accum - post_curve_ignore_start;
                if (ignore_delta < 0) ignore_delta = -ignore_delta;
                if (ignore_delta >= (int32)curve_exit_ignore_pulses)
                {
                    post_curve_ignore = 0;
                }
                junction_rising = 0;
            }
        }
        if (junction_rising)
        {
            if (target == 2)
            {
                // 十字直行：立刻推进 dir_count，进入忽略窗口
                state = 2;
                latched_encoder_exit = 0;
                encoder_exit_start = cross_encoder_accum;
                road_type = straight;
                if (dir_count + 1 < road_num)
                {
                    dir_count++;
                    dir_advance_pending = 1;
                }
            }
            else
            {
                // 0/1：循边线，yaw退弯；3/4：循边线，编码器阈值退弯
                latched_dir        = (target == 0 || target == 3) ? left : right;
                latched_encoder_exit = (target == 3 || target == 4) ? 1 : 0;
                state              = 1;
                curve_entry_yaw    = yaw_angle;
                curve_frame_count  = 0;
                yaw_complete_count = 0;
                encoder_exit_start = cross_encoder_accum;
                road_type          = latched_dir;
            }
        }
        break;

    case 1: // ===== STATE_CURVE：循 latched_dir，yaw 驱动退弯 =====
        curve_frame_count++;
        road_type = latched_dir;
        {
            if (latched_encoder_exit)
            {
                int32 encoder_delta = cross_encoder_accum - encoder_exit_start;
                if (encoder_delta < 0) encoder_delta = -encoder_delta;

                if (encoder_delta >= (int32)cross_ignore_pulses)
                {
                    state              = 0;
                    latched_dir        = straight;
                    latched_encoder_exit = 0;
                    yaw_complete_count = 0;
                    road_type          = straight;
                    post_curve_ignore  = (curve_exit_ignore_pulses > 0) ? 1 : 0;
                    post_curve_ignore_start = cross_encoder_accum;
                    control_pid_reset();
                    if (dir_count + 1 < road_num)
                    {
                        dir_count++;
                        dir_advance_pending = 1;
                    }
                }
                break;
            }

            float yaw_delta = fabsf(normalize_angle(yaw_angle - curve_entry_yaw));
            uint8 yaw_min_met = (yaw_delta >= MIN_CURVE_YAW_DEG);

            if (curve_frame_count >= CURVE_MIN_HOLD_FRAMES && yaw_min_met)
            {
                if (yaw_complete_count < 255) yaw_complete_count++;
                if (yaw_complete_count >= YAW_COMPLETE_STABLE_FRAMES)
                {
                    // 正常退弯：yaw 达标
                    state              = 0;
                    latched_dir        = straight;
                    latched_encoder_exit = 0;
                    yaw_complete_count = 0;
                    road_type          = straight;
                    post_curve_ignore  = (curve_exit_ignore_pulses > 0) ? 1 : 0;
                    post_curve_ignore_start = cross_encoder_accum;
                    control_pid_reset();
                    if (dir_count + 1 < road_num)
                    {
                        dir_count++;
                        dir_advance_pending = 1;
                    }
                }
            }
            else
            {
                yaw_complete_count = 0;
                if (!yaw_min_met && curve_frame_count >= CURVE_TIMEOUT_FRAMES)
                {
                    // 超时 + yaw 不足 → 伪弯道，强制退出，不推进 dir_count
                    state              = 0;
                    latched_dir        = straight;
                    latched_encoder_exit = 0;
                    yaw_complete_count = 0;
                    road_type          = straight;
                    post_curve_ignore  = (curve_exit_ignore_pulses > 0) ? 1 : 0;
                    post_curve_ignore_start = cross_encoder_accum;
                }
            }
        }
        break;

    case 2: // ===== STATE_CROSS：十字直行穿越，编码器忽略窗口 =====
        road_type = straight;
        {
            // 使用编码器ISR中的全局累计量，避免图像帧率与编码器采样频率不一致导致重复/漏计。
            int32 encoder_delta = cross_encoder_accum - encoder_exit_start;
            if (encoder_delta < 0) encoder_delta = -encoder_delta;

            if (encoder_delta >= (int32)cross_ignore_pulses)
            {
                // 忽略窗口结束，回到 STATE_STRAIGHT
                state = 0;
            }
        }
        break;

    default:
        state = 0;
        latched_encoder_exit = 0;
        road_type = straight;
        break;
    }

    cross_active = (state == 2 || post_curve_ignore) ? 1 : 0;

    // ===== 丢线保护停车（已禁用） =====
    // {
    //     static uint16 line_lost_frames = 0;
    //     uint8 both_lost = (left_edge_count < LINE_LOST_MIN_POINTS &&
    //                        right_edge_count < LINE_LOST_MIN_POINTS) ? 1 : 0;
    //     if (both_lost && state != 2)  // STATE_CROSS 期间豁免
    //     {
    //         if (line_lost_frames < 65535) line_lost_frames++;
    //         if (line_lost_frames >= LINE_LOST_STOP_FRAMES)
    //         {
    //             car_running = 0;
    //         }
    //     }
    //     else
    //     {
    //         line_lost_frames = 0;
    //     }
    // }
}

//===================================================================================================================
// 函数简介     初始化逆透视变换LUT
// 参数说明     void
// 返回参数     void
// 使用示例     init_warp_lut()
// 备注信息     放在init最后进行初始化，预计算所有输出像素对应的源图像坐标，运行时只需查表
//===================================================================================================================
void init_warp_lut(void)
{
    for (int i = 0; i < WARP_IMAGE_H; i++)
    {
        for (int j = 0; j < WARP_IMAGE_W; j++)
        {
            Transform_Point1(j, i);

            int x = (int)Tx;
            int y = (int)Ty;

            if (x >= 0 && x < IMG_W && y >= 0 && y < IMG_H)
            {
                warp_lut[i][j] = (uint16)(y * IMG_W + x);
            }
            else
            {
                warp_lut[i][j] = WARP_LUT_INVALID;
            }
        }
    }

    // 一次性初始化 warp_image：LUT 失效区（WARP_LUT_INVALID）永远不被覆盖，
    // 一次置 0 后每帧 WarpPerspective 只需覆写有效像素，极节省 ~10 μs/帧。
    memset(&warp_image[0][0], 0, WARP_IMAGE_H * WARP_IMAGE_W);
}

//===================================================================================================================
// 函数简介     逆透视变换坐标点（正向
// 参数说明     x y     变换点的坐标
// 返回参数     void
// 使用示例     Transform_Point1(x, y)
// 备注信息
//===================================================================================================================
void Transform_Point1(int x, int y)
{
    // 齐次坐标w分量
    float w = 1;

    // 矩阵变换计算
    float transformedX = Mat1[0][0] * x + Mat1[0][1] * y + Mat1[0][2] * w;
    float transformedY = Mat1[1][0] * x + Mat1[1][1] * y + Mat1[1][2] * w;
    float transformedW = Mat1[2][0] * x + Mat1[2][1] * y + Mat1[2][2] * w;

    // 齐次坐标归一化
    if (transformedW > 0.001f)
    {
        transformedX /= transformedW;
        transformedY /= transformedW;
    }
    else
    {
        // 变换无效，返回-1
        Tx = -1;
        Ty = -1;
        return;
    }

    // 保存变换结果
    Tx = transformedX;
    Ty = transformedY;
}

//===================================================================================================================
// 函数简介     逆透视变换图像
// 参数说明     void
// 返回参数     void
// 使用示例     WarpPerspective()
// 备注信息     LUT查表，无浮点运算
//===================================================================================================================
void WarpPerspective(void)
{
    uint8 *src = &mt9v03x_image[0][0];
    uint8 *dst = &warp_image[0][0];
    // memset 已迁移到 init_warp_lut：LUT 失效区位置固定，一次置 0 后每帧仅覆写有效像素
    for (int i = 0; i < WARP_IMAGE_H; i++)
    {
        uint16 *lut_row = warp_lut[i];
        uint8 *dst_row = dst + i * WARP_IMAGE_W;
        for (int j = 0; j < WARP_IMAGE_W; j++)
        {
            uint16 offset = lut_row[j];
            if (offset != WARP_LUT_INVALID)
                dst_row[j] = src[offset];
        }
    }
}

//===================================================================================================================
// 函数简介     图像处理主函数
// 参数说明     void
// 返回参数     void
// 使用示例     img_process()
// 备注信息     流程：逆透视变换 → 自适应边线检测 → 路口检测 → 方向判断 → 丢线检测
//===================================================================================================================
void img_process(void)
{
    //计时
    uint32 start_time = system_getval_us();


    // 逆透视变换（从原图直接读取灰度值）
    WarpPerspective();

    // 自适应边线检测（直接在灰度图上工作）
    zi_shi_ying_warp();

    // 路口检测（仅基于检测框）
    detect_junction_type();

    // 道路方向综合判断（路口+choose，输出RUN_Dir）
    detect_road_type();

    // ======================================

    //计时结束
    uint32 end_time = system_getval_us();
    img_process_time = (vuint16)(end_time - start_time);

}

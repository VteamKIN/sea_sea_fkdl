/*
 * img_process.c
 * 阿巴实现
 * 主要功能：我是奶龙
 * Created on: 2026年3月31日
 *      Author: aaa
 */

#include "zf_common_headfile.h"

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
int8  choose[road_num] = {1,0,1,1,1,1,-1};  // 弯道方向选择 0:循左线 1:循右线 -1:停止
uint16 curve_lockout_ms[road_num] = {5,200,3,5,0,1,50,200,2,3,2,3,4,6,0,10}; // 每个弯道退弯后锁定帧数（独立设置）
vint8 current_target_dir = 0;        // 当前循线方向(choose[dir_count]的缓存)
vuint8 dir_advance_pending = 0;      // dir_count 推进事件信号（detect_road_type 退弯瞬间设 1，control.c 处理后清 0）

volatile RUN_Dir road_type =straight;       // 道路类型 straight/left/right
volatile enum JunctionType current_junction = JUNCTION_NONE;  // 当前路口类型
vuint8  road_width_avg = 0;          // 平均道路宽度(像素)
vuint8  junction_detected = 0;       // 路口检测标志 0:无 1:有
volatile enum JunctionType raw_junction_debug = JUNCTION_NONE; // 路口检测原始值（供调试显示）
volatile uint8 left_slope_mutation = 0;      // 左边线是否检测到斜率突变（方案E输出，调试可见）
volatile uint8 right_slope_mutation = 0;     // 右边线是否检测到斜率突变（方案E输出，调试可见）
static enum JunctionType last_raw_junction = JUNCTION_NONE;   // 上一次原始路口检测结果(防抖用)
static uint8 junction_stable_count = 0;      // 路口检测防抖计数

static float Mat1[3][3]= {  { 0.616450095074508, -0.457607373643385, 35.7432540391721},
                            { 0.011114605002624, 0.666532477149147, 5.98414122411157},
                            { -0.000199978688494178, -0.00567658803479972, 0.942310105378709}, };

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

        int rs = -1, rl = 0;// 当前亮段的起始 x 坐标和长度
        int b_start = -1, b_len = 0, b_dist = img_w;

        for (int x = 0; x < img_w; x++)
        {
            if (row[x] > row_thres)// 找第一个大于阈值的点 更新rl
            {
                if (rs == -1) rs = x;
                rl++;
            }
            else
            {
                if (rl >= NARROW_WIDTH_MIN)// 道路宽度判断，大于才取
                {
                    int rc = rs + rl / 2;// 计算道路的中心
                    int d = rc - last_center;// 偏移量
                    if (d < 0) d = -d;
                    if (d < b_dist) { b_dist = d; b_start = rs; b_len = rl; }//道路中心的合法判断
                }
                rs = -1;
                rl = 0;
            }
        }
        // 处理行尾亮段 亮段一直延伸到最后
        if (rl >= NARROW_WIDTH_MIN)
        {
            int rc = rs + rl / 2;
            int d = rc - last_center;
            if (d < 0) d = -d;
            if (d < b_dist) { b_start = rs; b_len = rl; }
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

            int rs = -1, b_start = -1, b_len = 0, rl = 0;
            for (int x = 0; x < img_w; x++)
            {
                if (row[x] > t)
                {
                    if (rs == -1) rs = x;
                    rl++;
                    if (rl > b_len) { b_len = rl; b_start = rs; }
                }
                else { rs = -1; rl = 0; }
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
        return 0;

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
// 函数简介     路口检测
// 参数说明     void
// 返回参数     void
// 使用示例     detect_junction_type()
// 备注信息     路宽判断，防止弯道进入图像顶部时整行都是道路导致误判
//===================================================================================================================
void detect_junction_type(void)
{
    const int img_w = WARP_IMAGE_W;
    const uint8 *img_data = &warp_image[0][0];

    int thres = global_warp_thres;

    // 先计算底部几行的道路宽度
    int base_width = 0;
    int base_count = 0;
    for (int y = end_point_y; y > end_point_y - 5 && y >= start_point_y; y--)
    {
        int row_left = -1, row_right = -1;
        for (int x = start_point_x; x <= end_point_x; x++)
        {
            if (img_data[y * img_w + x] > thres)
            {
                if (row_left == -1) row_left = x;
                row_right = x;
            }
        }
        if (row_left != -1 && row_right != -1)
        {
            base_width += row_right - row_left + 1;
            base_count++;
        }
    }
    if (base_count > 0)
        base_width /= base_count;
    road_width_avg = (uint8)base_width;

    // 四边道路到达边界的标志

    uint8 road_top = 0;
    uint8 road_bottom = 0;
    uint8 road_left = 0;
    uint8 road_right = 0;

    // 路宽范围：连续道路像素在[下限, 上限)范围内才视为有路
    // 下限：排除噪声和小面积干扰；上限：排除弯道（弯道水平段会超出赛道宽度）
    int road_pixel_min = ROAD_PIXEL_THRESHOLD;
    int road_pixel_max = (int)(road_width_avg * 2);
    if (road_pixel_max < ROAD_PIXEL_THRESHOLD * 2)
        road_pixel_max = ROAD_PIXEL_THRESHOLD * 2;

    // 道路段起止坐标（用于弯道相连判断）
    int top_road_start = -1, top_road_end = -1;
    int bottom_road_start = -1, bottom_road_end = -1;
    int left_road_start = -1, left_road_end = -1;
    int right_road_start = -1, right_road_end = -1;

    // 扫描上边 (y = start_point_y)
    int consecutive = 0;
    int max_consecutive = 0;
    int seq_start = -1;
    int max_seq_start = -1;
    for (int x = start_point_x; x <= end_point_x; x++)
    {
        if (img_data[start_point_y * img_w + x] > thres)
        {
            if (consecutive == 0) seq_start = x;
            consecutive++;
            if (consecutive > max_consecutive)
            {
                max_consecutive = consecutive;
                max_seq_start = seq_start;
            }
        }
        else
        {
            consecutive = 0;
        }
    }
    if (max_consecutive >= road_pixel_min &&
        max_consecutive < road_pixel_max)
    {
        road_top = 1;
        top_road_start = max_seq_start;
        top_road_end = max_seq_start + max_consecutive - 1;
    }

    // 扫描下边 (y = end_point_y)
    consecutive = 0;
    max_consecutive = 0;
    seq_start = -1;
    max_seq_start = -1;
    for (int x = start_point_x; x <= end_point_x; x++)
    {
        if (img_data[end_point_y * img_w + x] > thres)
        {
            if (consecutive == 0) seq_start = x;
            consecutive++;
            if (consecutive > max_consecutive)
            {
                max_consecutive = consecutive;
                max_seq_start = seq_start;
            }
        }
        else
        {
            consecutive = 0;
        }
    }
    // 下边不加上限限制
    if (max_consecutive >= road_pixel_min)
    {
        road_bottom = 1;
        bottom_road_start = max_seq_start;
        bottom_road_end = max_seq_start + max_consecutive - 1;
    }

    // 扫描左边 (x = start_point_x)
    consecutive = 0;
    max_consecutive = 0;
    seq_start = -1;
    max_seq_start = -1;
    for (int y = start_point_y; y <= end_point_y; y++)
    {
        if (img_data[y * img_w + start_point_x] > thres)
        {
            if (consecutive == 0) seq_start = y;
            consecutive++;
            if (consecutive > max_consecutive)
            {
                max_consecutive = consecutive;
                max_seq_start = seq_start;
            }
        }
        else
        {
            consecutive = 0;
        }
    }
    if (max_consecutive >= road_pixel_min &&
        max_consecutive < road_pixel_max)
    {
        road_left = 1;
        left_road_start = max_seq_start;
        left_road_end = max_seq_start + max_consecutive - 1;
    }

    // 扫描右边 (x = end_point_x)
    consecutive = 0;
    max_consecutive = 0;
    seq_start = -1;
    max_seq_start = -1;
    for (int y = start_point_y; y <= end_point_y; y++)
    {
        if (img_data[y * img_w + end_point_x] > thres)
        {
            if (consecutive == 0) seq_start = y;
            consecutive++;
            if (consecutive > max_consecutive)
            {
                max_consecutive = consecutive;
                max_seq_start = seq_start;
            }
        }
        else
        {
            consecutive = 0;
        }
    }
    if (max_consecutive >= road_pixel_min &&
        max_consecutive < road_pixel_max)
    {
        road_right = 1;
        right_road_start = max_seq_start;
        right_road_end = max_seq_start + max_consecutive - 1;
    }


    detect_edge_slope_mutation();

    // 弯道相连判断：同一道路在两个边界的投影不应视为两个方向都有路
    if (road_top && road_left)
    {
        if (top_road_start <= start_point_x + 5 && left_road_start <= start_point_y + 5)
        {
            if (!left_slope_mutation)
            {
                road_top = 0;
                road_left = 0;
            }
        }
    }
    // 上+右相连：同理，用右边线突变消解
    if (road_top && road_right)
    {
        if (top_road_end >= end_point_x - 5 && right_road_start <= start_point_y + 5)
        {
            if (!right_slope_mutation)
            {
                road_top = 0;
                road_right = 0;
            }
        }
    }
    // 下+左相连：直道偏左时道路同时碰到下边界和左边界，只清除侧边标志
    if (road_bottom && road_left)
    {
        if (bottom_road_start <= start_point_x + 5 && left_road_end >= end_point_y - 5)
        {
            road_left = 0;
        }
    }
    // 下+右相连：直道偏右时道路同时碰到下边界和右边界，只清除侧边标志
    if (road_bottom && road_right)
    {
        if (bottom_road_end >= end_point_x - 5 && right_road_end >= end_point_y - 5)
        {
            road_right = 0;
        }
    }

    // 原始路口判断（下方必须有路）
    enum JunctionType raw_junction = JUNCTION_NONE;
    if (road_bottom)
    {
        if (road_top && road_left && road_right)
            raw_junction = JUNCTION_CROSS;
        else if (road_left && road_right && !road_top)
            raw_junction = JUNCTION_T;
        else if (road_top && road_left && !road_right)
            raw_junction = JUNCTION_LEFT_T;
        else if (road_top && road_right && !road_left)
            raw_junction = JUNCTION_RIGHT_T;
        else if (road_left && !road_right && !road_top)
            raw_junction = JUNCTION_LEFT;
        else if (road_right && !road_left && !road_top)
            raw_junction = JUNCTION_RIGHT;
    }

    // 记录原始检测值供屏显调试（过滤前）
    raw_junction_debug = raw_junction;

    // 进弯抖动过滤
    // 短暂污染为错误类型
    {
        int8 _flt_dir = (dir_count < road_num) ? choose[dir_count] : -1;
        if ((raw_junction == JUNCTION_LEFT  && _flt_dir != 0) ||
            (raw_junction == JUNCTION_RIGHT && _flt_dir != 1))
        {
            raw_junction = JUNCTION_NONE;
        }
    }

    // 防抖
    {
        static uint8 mutation_ever_set = 0;
        uint8 raw_active  = (raw_junction      != JUNCTION_NONE) ? 1 : 0;
        uint8 last_active = (last_raw_junction != JUNCTION_NONE) ? 1 : 0;
        if (raw_active == last_active)
        {
            if (junction_stable_count < 255) junction_stable_count++;
            if (raw_active && (left_slope_mutation || right_slope_mutation))
                mutation_ever_set = 1;
            if (junction_stable_count >= JUNCTION_STABLE_THRESHOLD)
            {
                if (raw_active && !mutation_ever_set)
                {
                    // 连续 N 帧非 NONE 但窗口内从无突变 → 倾斜直道伪路口
                    current_junction = JUNCTION_NONE;
                    junction_detected = 0;
                }
                else
                {
                    current_junction = raw_junction;
                    junction_detected = raw_active;
                }
            }
        }
        else
        {
            junction_stable_count = 0;
            mutation_ever_set = 0;
        }
        last_raw_junction = raw_junction;
    }
}

//===================================================================================================================
// 函数简介     道路方向综合判断（Yaw 驱动退弯状态机）
// 参数说明     void
// 返回参数     void
// 使用示例     detect_road_type()
// 备注信息     转弯：yaw >= MIN_CURVE_YAW_DEG 退弯；直行T字：视觉退弯；超时保护防反馈环
//===================================================================================================================
void detect_road_type(void)
{
    // 边界检查
    if (dir_count >= road_num)
    {
        road_type = straight;
        return;
    }

    int8 target = choose[dir_count];
    current_target_dir = target;

    // 停车标志：road_type=straight，control.c 自行处理停车
    if (target == -1)
    {
        road_type = straight;
        return;
    }

    static uint8     in_curve              = 0;        // 0=STATE_STRAIGHT, 1=STATE_CURVE
    static RUN_Dir   latched_dir           = straight; // CURVE 状态下循的边线
    static float     curve_entry_yaw       = 0.0f;     // 进弯时的 yaw 角
    static uint16    curve_frame_count     = 0;         // CURVE 状态持续帧数
    static enum JunctionType curve_entry_junction = JUNCTION_NONE; // 进弯时路口类型
    static uint16    straight_count        = 0;         // 直行通过保持帧数
    static uint8     yaw_complete_count    = 0;

    if (!in_curve)
    {
        // ===== STATE_STRAIGHT：循中线，等待识别到弯道 =====
        if (junction_detected)
        {
            latched_dir           = (target == 0) ? left : right;
            in_curve              = 1;
            curve_entry_yaw       = yaw_angle;
            curve_frame_count     = 0;
            curve_entry_junction  = current_junction;
            straight_count        = 0;
            yaw_complete_count    = 0;
            road_type             = latched_dir;
        }
        else
        {
            road_type = straight;
        }
    }
    else
    {
        // ===== STATE_CURVE：循 latched_dir，等待退出 =====
        curve_frame_count++;
        road_type = latched_dir;
        float yaw_delta = fabsf(normalize_angle(yaw_angle - curve_entry_yaw));

        // 判断是否为直行通过（LEFT_T循右=直行，RIGHT_T循左=直行）
        uint8 is_straight_through =
            (curve_entry_junction == JUNCTION_LEFT_T  && target == 1) ||
            (curve_entry_junction == JUNCTION_RIGHT_T && target == 0);

        if (is_straight_through && yaw_delta < STRAIGHT_T_YAW_LIMIT_DEG)
        {
            // 直行通过：保持 N 帧后退弯
            straight_count++;
            if (straight_count >= EXIT_STRAIGHT_FRAMES)
            {
                in_curve       = 0;
                latched_dir    = straight;
                straight_count = 0;
                yaw_complete_count = 0;
                road_type      = straight;
                if (dir_count + 1 < road_num)
                {
                    dir_count++;
                    dir_advance_pending = 1;
                }
            }
        }
        else
        {
            // 转弯场景：双阈值 yaw 退弯
            uint8 yaw_min_met = (yaw_delta >= MIN_CURVE_YAW_DEG);

            if (curve_frame_count >= CURVE_MIN_HOLD_FRAMES && yaw_min_met)
            {
                if (yaw_complete_count < 255) yaw_complete_count++;
                if (yaw_complete_count >= YAW_COMPLETE_STABLE_FRAMES)
                {
                    // 正常退弯：yaw 达标
                    in_curve           = 0;
                    latched_dir        = straight;
                    straight_count     = 0;
                    yaw_complete_count = 0;
                    road_type          = straight;
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
                    in_curve           = 0;
                    latched_dir        = straight;
                    straight_count     = 0;
                    yaw_complete_count = 0;
                    road_type          = straight;
                }
            }
        }
    }

    // ===== 丢线保护停车 =====
    {
        static uint16 line_lost_frames = 0;
        uint8 both_lost = (left_edge_count < LINE_LOST_MIN_POINTS &&
                           right_edge_count < LINE_LOST_MIN_POINTS) ? 1 : 0;
        if (both_lost)
        {
            if (line_lost_frames < 65535) line_lost_frames++;
            if (line_lost_frames >= LINE_LOST_STOP_FRAMES)
            {
                car_running = 0;
            }
        }
        else
        {
            line_lost_frames = 0;
        }
    }
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

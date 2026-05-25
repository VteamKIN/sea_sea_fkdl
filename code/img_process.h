/*
 * img_process.h
 * 图像处理模块头文件
 * 主要功能：图像预处理、边线检测、道路识别、逆透视变换
 * Created on: 2026年3月31日
 *      Author: aaa
 */

#ifndef CODE_IMG_PROCESS_H_
#define CODE_IMG_PROCESS_H_

#include "zf_common_headfile.h"

// ============================================================
// 基本常量定义
// ============================================================

#define IMG_H MT9V03X_H              // 原始图像高度
#define IMG_W MT9V03X_W              // 原始图像宽度

#define Black  0                     // 黑色像素值
#define White 255                    // 白色像素值

#define WARP_IMAGE_W 124             // 逆透视后图像宽度
#define WARP_IMAGE_H 80              // 逆透视后图像高度（有效区域）
#define IMAGE_SIZE  (WARP_IMAGE_W * WARP_IMAGE_H)//逆透视后图像大小
// 检测区域
#define start_point_x 6             // 检测区域左边界
#define start_point_y 5             // 检测区域上边界
#define end_point_x   118            // 检测区域右边界
#define end_point_y   75             // 检测区域下边界

// 路口检测参数
#define ROAD_PIXEL_THRESHOLD  7      // 连续道路像素阈值（连续超过此值视为有路）

// ============================================================
// 枚举类型定义
// ============================================================

// 运行方向枚举
typedef enum RUN_Dir
{
    straight,
    left,
    right,
} RUN_Dir;

// 路口类型枚举
enum JunctionType {
    JUNCTION_NONE = 0,      // 无路口
    JUNCTION_CURVE,         // 弯道/路口（边线端点到达图像边界）
};

// ============================================================
// 自适应边线跟踪常量
// ============================================================

#define ZSY_BLOCK_SIZE     7
#define ZSY_CLIP_VALUE     5               // 窄赛道：正值，阈值=均值+clip（更严格区分黑白）
#define MAX_EDGE_POINTS    200              // 边线点最大数量
#define WARP_LUT_INVALID   0xFFFF          // 逆透视LUT无效标记


// 200 fps (5ms/帧) 标准参数集
#define JUNCTION_STABLE_THRESHOLD  2        // 路口检测防抖（进弯+退弯双向防抖）

// Yaw 驱动退弯参数
#define MIN_CURVE_YAW_DEG       75.0f      // 主阈值：yaw 累计达此值退弯
#define CURVE_TIMEOUT_FRAMES    160        // CURVE 超时(帧)：800ms @200fps
#define CURVE_MIN_HOLD_FRAMES   5          // 进弯后最少保持帧数
#define YAW_COMPLETE_STABLE_FRAMES 3       // yaw 达标后连续确认帧数

// 十字路口直行穿越参数
#define CROSS_IGNORE_PULSES_DEFAULT  2050    // 默认穿越忽略窗口（编码器脉冲累计值）

// 边线斜率突变检测（方案 E）：过滤倾斜直道误识别为路口
// 原理：直道（含倾斜）边线沿自身斜率恒定无突变；真路口边线在某点急转，斜率突变大
#define EDGE_SLOPE_NEAR_IDX_LO    2        // 近段起点索引（跳过最近 2 点抖动）
#define EDGE_SLOPE_NEAR_IDX_HI    12       // 近段终点索引
#define EDGE_SLOPE_FAR_IDX_LO     15       // 远段起点索引
#define EDGE_SLOPE_FAR_IDX_HI     25       // 远段终点索引
#define EDGE_MIN_POINTS           28       // 边线最少点数（保证可取到远段索引）
#define SLOPE_CHANGE_THRESHOLD    100      // 斜率突变阈值（×100），100 ≈ 45° 角度变化
                                           // 小于此值 → 边线沿固定方向 → 倾斜直道，过滤为 NONE

// 底部起点检测参数
#define BOTTOM_SAMPLE_ROWS        8        // 底部参与起点检测的行数
#define ROAD_MIN_RUN_PIXELS       3        // 认为是道路的最小连续亮像素数
#define NARROW_WIDTH_MIN          4        // 窄赛道允许的最小道路宽度(像素)
#define START_MIN_RUN             3        // 起点检测最小连续亮长度
#define START_GAP_ALLOW           2        // 起点检测允许的最大中断像素数
#define ROAD_THRES_CLIP           3        // 阈值留白(减小提高检出率)
#define MIN_ROAD_CONTRAST         15       // 起点检测最小对比度(行最大-最小)，低于此值视为无赛道
#define CBH_START_SHIFT           3        // 差比和边缘检测的像素偏移
#define CBH_RATIO_THRESHOLD       15       // 差比和跳变阈值(放大128倍,降低提高弱边缘检出)
#define CBH_WHITE_OFFSET          3        // 判定白区时相对行阈值的增量(降低提高暗道检出)

// 边线回折截断阈值
#define EDGE_FOLD_THRESHOLD       4        // 连续y递增超过此点数视为回折，截断后续点

// 元器件中空间隙跨越（起点检测用）
#define ELEMENT_GAP_MAX_DEFAULT   15       // 起点检测允许跨越的最大暗间隙(像素)
                                           // 元器件中空宽度通常 < 路宽的 3/4

// 丢线保护停车
#define LINE_LOST_MIN_POINTS      3        // 边线点数低于此值视为丢线
#define LINE_LOST_STOP_FRAMES     50       // 连续丢线帧数达到此值则停车（~250ms @200fps）

// ============================================================
// 图像数据结构
// ============================================================

typedef struct {
    uint8 *data;
    int16 width;
    int16 height;
} image_t;

#define AT_IMAGE(img, x, y)  ((img)->data[(y) * (img)->width + (x)])
#define DEF_IMAGE(d, w, h)   ((image_t){(d), (w), (h)})


// ============================================================
// 全局变量声明（extern）
// ============================================================

// --- 边线点列表 ---
extern int16 left_edge[MAX_EDGE_POINTS][2];   // 左边线坐标 [i][0]=x [i][1]=y
extern vuint8 left_edge_count;        // 左边线点数
extern int16 right_edge[MAX_EDGE_POINTS][2];  // 右边线坐标 [i][0]=x [i][1]=y
extern vuint8 right_edge_count;       // 右边线点数

// --- 道路类型识别变量 ---
#define road_num 20
extern vuint8  dir_count;             // 当前弯道序号(0~19)
extern int8   choose[road_num];              // 方向选择 0:循左线 1:循右线 2:直行穿越 -1:停止
extern int16  cross_ignore_pulses;           // 十字直行忽略窗口（编码器脉冲，可调参）
extern vint8 current_target_dir;      // 当前循线方向(choose[dir_count]的缓存)
extern volatile RUN_Dir road_type;            // 道路类型 straight/left/right
extern vuint8 dir_advance_pending;    // dir_count 推进事件信号：detect_road_type 退弯瞬间设 1，control.c 处理后清 0

// --- 路口检测变量 ---
extern volatile enum JunctionType current_junction;  // 当前路口类型
extern volatile enum JunctionType raw_junction_debug; // 路口检测原始值（调试用）
extern vuint8 left_slope_mutation;            // 左边线是否检测到斜率突变
extern vuint8 right_slope_mutation;           // 右边线是否检测到斜率突变
extern vuint8  road_width_avg;        // 平均道路宽度(像素)
extern vuint8  junction_detected;     // 路口检测标志 0:无 1:有

// --- 性能计时变量 ---
extern vuint16 img_process_time;      // 图像处理耗时(μs)

// --- 行前缀和 ---
extern uint16 row_prefix[WARP_IMAGE_H][WARP_IMAGE_W + 1];  // 行前缀和数组

// --- 全局阈值变量 ---
extern vuint8 global_warp_thres;      // 逆透视图全局二值化阈值

// --- 逆透视变换变量 ---
extern uint8  warp_image[80][124];            // 逆透视图像缓冲
extern uint8  *warp_image_ptr;                // 逆透视图像指针(供TFT/Send使用)

// ============================================================
// 函数声明
// ============================================================

// --- 自适应边线跟踪 ---
void build_row_prefix(image_t *img);
void findline_lefthand_adaptive(image_t *img, int block_size, int clip_value,

                                int x, int y, int16 pts[][2], int16 *num);
void findline_righthand_adaptive(image_t *img, int block_size, int clip_value,
                                 int x, int y, int16 pts[][2], int16 *num);
// void zi_shi_ying(uint8 *My_Image);  // 已移除
void zi_shi_ying_warp(void);

// --- 道路类型识别 ---
void detect_edge_slope_mutation(void);  // 边线斜率突变检测（写入 left/right_slope_mutation）
void detect_junction_type(void);  // 路口检测（基于检测框，综合斜率突变结果）
void detect_road_type(void);         // 道路方向综合判断（路口+choose，更新 road_type）

// --- 逆透视变换 ---
void init_warp_lut(void);         // 初始化LUT查找表（启动时调用一次）
void Transform_Point1(int x, int y);
void WarpPerspective(void);

// --- 综合处理入口 ---
void img_process(void);

#endif /* CODE_IMG_PROCESS_H_ */

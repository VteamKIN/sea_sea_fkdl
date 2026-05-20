---
name: port-cha_bi_he-zi_shi_ying
overview: 将待移植/cha_bi_he.c 和待移植/zi_shi_ying.c 的增量代码移植到 code/control_process.c/.h 中，复用已有变量定义，删除重复项，补充缺失依赖。
todos:
  - id: modify-control-process-h
    content: 在 control_process.h 中添加 CBH/自适应算法的宏定义、image_t 结构体、extern 声明和函数声明，复用 IMG_W/IMG_H
    status: completed
  - id: add-cbh-to-c
    content: 在 control_process.c 中添加差比和算法全局变量和全部函数实现，修正 Bug，删除冗余 abs()
    status: completed
    dependencies:
      - modify-control-process-h
  - id: add-zi-shi-ying-to-c
    content: 在 control_process.c 中添加自适应跟踪算法全局变量、image_t 适配层和全部函数实现
    status: completed
    dependencies:
      - modify-control-process-h
  - id: verify-no-conflicts
    content: 检查 lint 错误，确认无符号重复、无链接冲突、宏展开正确
    status: completed
    dependencies:
      - add-cbh-to-c
      - add-zi-shi-ying-to-c
---

## 需求概述

将 `待移植/` 目录下的差比和巡线算法（`cha_bi_he.c/.h`）和自适应局部阈值边线跟踪算法（`zi_shi_ying.c/.h`）移植到目标环境 `code/control_process.c/.h` 中。移植时必须：

1. 复用目标环境已有的变量、函数定义及配置
2. 识别并删除重复定义或冗余部分
3. 仅保留必要的增量代码

## 核心要求

- 将差比和（CBH）算法和自适应跟踪算法作为可选的图像处理手段整合到现有 `img_process()` 流程中
- 修正源代码中已发现的 Bug（如 `CAM_WIDTH-2` 误用为行号）
- 补充目标环境缺失的依赖定义（`image_t`、`AT_IMAGE`、`DEF_IMAGE`、`xun_xian_cs`）
- 解决图像尺寸不一致问题（源码 `CAM_HEIGHT=120` vs 目标 `MT9V03X_H=128`）

## 技术方案

### 关键冲突与解决策略

| 冲突项 | 源代码 | 目标环境 | 解决方案 |
| --- | --- | --- | --- |
| 图像尺寸宏 | `CAM_WIDTH=160, CAM_HEIGHT=120` | `IMG_W=160, IMG_H=128` | 删除 CAM_* 宏，全部使用 IMG_W/IMG_H |
| 图像数组 | `My_Image_Copy[120][160]` | `imgGray[128][160]` | 删除 My_Image_Copy，直接使用 imgGray |
| 边界数组 | `lift_line[], right_line[], mid_Line[]` | `left_boundary[], right_boundary[], center_line[]` | 保留 CBH 专用数组（类型为 uint8 vs int 不同，且算法逻辑独立） |
| 限幅函数 | `INT_xian_fu()` | `abs()` | 保留 INT_xian_fu（双向限幅），删除与标准库冲突的 `abs()` |
| abs() | 无 | `control_process.c` L74 | 删除自定义 `abs()`，已有 `<stdlib.h>` 提供 |
| 图像结构体 | `image_t`, `AT_IMAGE`, `DEF_IMAGE` | 不存在 | 在 control_process.h 中定义最小化版本 |
| 巡线点数 | `xun_xian_cs` 未定义 | 不存在 | 定义为 `#define XUN_XIAN_CS 200` |
| 函数名冲突 | `tu_xiang()` | 无直接冲突 | 重命名为 `cbh_img_process()` 表意更清晰 |
| Bug 修复 | `cha_bi_he_saoxian(CAM_WIDTH-2, ...)` | -- | 修正为 `IMG_H - 2`（行号应基于高度非宽度） |
| 方向数组名 | `cbh_fangx` | `guize`（八邻域） | 保留，四邻域和八邻域是不同算法 |


### 架构设计

两个新算法作为独立模块整合到 control_process 中，与现有的 OTSU+八邻域算法并存，用户可按需调用：

```
img_process() [现有主流程: OTSU → 八邻域 → 边界填充 → 道路检测]
    |
    +-- cbh_img_process() [新增: CBH差比和扫线流程]
    |       |-- get_reference_point()     自适应阈值
    |       |-- get_zui_chang_bai_lie()   找最长白列
    |       +-- cha_bi_he_saoxian()       差比和扫线
    |
    +-- zi_shi_ying() [新增: 自适应边线跟踪流程]
            |-- 左侧起始点搜索 → findline_lefthand_adaptive()
            +-- 右侧起始点搜索 → findline_righthand_adaptive()
```

### 目录结构

```
code/
├── control_process.h   # [MODIFY] 添加新宏定义、image_t 结构体、函数声明
├── control_process.c   # [MODIFY] 添加新全局变量、函数实现，删除冗余代码
```

### 实施细节

#### control_process.h 修改内容

1. 在现有宏定义区域后添加 CBH 算法参数宏（`CBH_WEIYI`、`End_Jiezhi`、`Start_JIezhi` 等），所有尺寸相关宏改用 `IMG_W`/`IMG_H`
2. 添加 `image_t` 结构体定义和 `AT_IMAGE`/`DEF_IMAGE` 宏（最小化定义，仅含 `data` 指针、`width`、`height`）
3. 添加 `XUN_XIAN_CS` 常量定义
4. 添加自适应跟踪参数宏（`BLOCK_SIZE_DEFAULT`、`CLIP_VALUE_DEFAULT`）
5. 添加所有新全局变量的 `extern` 声明
6. 添加所有新函数的声明

#### control_process.c 修改内容

1. **删除** `int abs(int num)` 函数（L74-78），标准库已提供
2. **添加** CBH 全局变量（`lift_line[]`、`right_line[]`、`mid_Line[]`、`White_Tallst_Point` 等）
3. **添加** 自适应跟踪全局变量（`ipts0[][]`、`ipts1[][]`、方向数组等）
4. **添加** `INT_xian_fu()`、`FLOAT_xian_fu()`、`clip()` 辅助函数
5. **添加** CBH 算法函数：`get_reference_point()`、`cha_bi_he()`、`get_zui_chang_bai_lie()`、`cha_bi_he_saoxian()`、`cbh_img_process()`
6. **添加** 自适应跟踪函数：`findline_lefthand_adaptive()`、`findline_righthand_adaptive()`、`zi_shi_ying()`
7. **修正** `tu_xiang()` 中的 `CAM_WIDTH-2` → `IMG_H-2`，并重命名为 `cbh_img_process()`
8. **修正** `zi_shi_ying()` 中的 `begin_y=114` → `IMG_H-14`，使起始点自适应图像高度
9. 所有函数内部将 `CAM_WIDTH` → `IMG_W`、`CAM_HEIGHT` → `IMG_H`
10. `My_Image_Copy[0]` 参数替换为 `image_gray`（已有全局灰度图指针）

### 性能考量

- CBH 算法计算量低（逐像素比较），适合实时性要求高的场景
- 自适应跟踪每个像素点需计算 `block_size x block_size` 窗口均值，计算量较高（O(N x block_size^2)），建议仅在需要时调用或作为备用算法
- 两个算法共享同一个 `imgGray`/`image_gray` 图像缓冲区，无需额外内存
- CBH 专用数组（`lift_line[]` 120 bytes x3）和跟踪点数组（`ipts0/1` 200x2x4 bytes）占用 RAM 约 2KB，在 TC264D 64KB+ RAM 环境中可接受
---
name: fix-offset-detection-and-adapt-tracking
overview: 修复车辆偏移时路口/角点检测误判问题；将边线跟踪算法从大赛道适配到小赛道（调整turn限制、clip_value符号、增加偏移感知）
todos:
  - id: adapt-tracking-algo
    content: "修改findline_lefthand_adaptive和findline_righthand_adaptive: clip_value改加法、turn改为4、右手边界条件修正"
    status: completed
  - id: update-tracking-constants
    content: 更新img_process.h中ZSY_CLIP_VALUE为正值5, 并更新相关注释
    status: completed
  - id: improve-corner-detect
    content: "改进detect_corners: 增加基于边线总点数的比例判断，防止偏移时误判角点"
    status: completed
    dependencies:
      - adapt-tracking-algo
  - id: improve-junction-detect
    content: "改进detect_junction_type: 增加偏移状态感知，对单侧road_left/right提高判定门槛"
    status: completed
    dependencies:
      - adapt-tracking-algo
  - id: improve-lost-detect
    content: "改进detect_lost: 放宽边界容差，减少偏移时的丢线误判"
    status: completed
    dependencies:
      - adapt-tracking-algo
---

## 产品概述

修复智能车在车辆偏移时路口/角点检测误判的问题，并将边线跟踪算法从大赛道适配为窄赛道版本。

## 核心功能

### 问题1：车辆偏移时检测出错

当车辆较偏（道路接近图像边缘）时，三个检测模块均使用**绝对坐标阈值**判断，未考虑偏移状态：

- **角点检测** (`detect_corners`)：固定用 `LEFT_EXTEND_THRESHOLD=8` / `RIGHT_EXTEND_THRESHOLD=116` 判断边线是否延伸到边界。车辆偏右时左边线整体右移但可能仍 ≤8 → 误判左角点；偏左时同理。
- **路口检测** (`detect_junction_type`)：固定检测框 `[start_point_x=6, end_point_x=118]`，偏移时道路触及框边缘 → 误判 `road_left`/`road_right`。
- **丢线检测** (`detect_lost`)：固定 `start_point_x+2` / `end_point_x-2` 判断边界，偏移时误判丢线。

### 问题2：边线跟踪不适配窄赛道

当前算法借鉴自大赛道代码(`zi_shi_ying.c`)，与参考小赛道代码存在4个关键差异：

| 参数 | 当前(大赛道) | 参考(小赛道) | 影响 |
| --- | --- | --- | --- |
| `local_thres` 计算 | `-= clip_value`(减法) | `+= clip_value`(加法) | **阈值方向完全相反！**减法使阈值偏低，灰色过渡区被当成白色道路→跟踪"飘"出边界 |
| `turn <` | 12 | 4 | 窄赛道弯道更急促，turn=12导致跟踪穿透窄路边界 |
| 左手边界条件 | `half < x` (即 x>=4) | `x >= half` (等价) | 基本一致 |
| 右手边界条件 | `0 < x` (太松) | `x >= half` (严格) | 右手允许x=1/2/3，block_size=7时会越界访问 |


## Tech Stack

- 语言：C语言（GBK编码），TC264/AURIX嵌入式平台
- 图像尺寸：WARP_IMAGE_W=124, WARP_IMAGE_H=80
- 修改范围：仅 img_process.c 和 img_process.h（不涉及libraries）

## Implementation Approach

### 核心策略：分两层改进

1. **底层**：修正边线跟踪算法参数，使其适配窄赛道（这是根本——跟踪准了，后续所有检测的基础数据才可靠）
2. **上层**：让角点/路口/丢线检测具备偏移感知能力，不依赖绝对坐标硬阈值

### 边线跟踪修改（参考小赛道代码，但保持独立思考）

**关于 clip_value 方向的关键分析：**

- 当前：`local_thres = 均值 - 9` → 阈值偏低 → 更多像素被判为"白(道路)" → 跟踪容易飘出边界
- 参考：`local_thres = 均值 + 正数` → 阈值偏高 → 更严格区分黑白 → 更紧贴真实边界
- 但注意：如果 clip_value 设得太大（如+20），可能导致正常道路内部的像素也被判黑 → 跟踪提前终止
- **决策**：采用参考代码的加法方向，但值设为较小的正值（如5），兼顾严格性和鲁棒性

**关于 turn<4 vs turn<12：**

- 窄赛道图像124×80，道路宽度约30~60像素
- turn=12 意味着允许连续12次90度转向（3圈）才停止——这在窄赛道上会穿透边界
- turn=4 意味着最多1圈转向——更适合窄赛道
- **决策**：改为 turn<4

**关于右手边界条件：**

- 当前 `0 < x` 允许x取到1,2,3，但 block_size=7 时 half=3，访问 x±dx(dx最大为3) 会到 x=-2 → 数组越界
- **决策**：改为 `x >= half`

### 角点检测改进

- 引入**比例判断**：延伸点数占该边线总点数的比例超过阈值(如25%)才判定为角点
- 同时保留绝对坐标阈值作为辅助条件（两者满足其一即可）
- 这样即使车辆偏移，只要不是真正的角点（延伸段占比小），就不会误判

### 路口检测改进

- 在 `road_left`/`road_right` 判定中增加**排除偏移**逻辑：如果检测框底部道路宽度正常(>15px)且 `road_bottom=1`，则单侧的 road_left/road_right 更可能是偏移导致的而非真实岔路
- 具体策略：当只有 road_left 或 road_right 单独置位（无road_top）时，要求对应侧的道路到达边界的连续长度更长（从5提高到8），以过滤偏移导致的边缘触碰

### 丢线检测改进

- 将绝对坐标阈值替换为基于 `road_width_avg` 的动态阈值
- 例如：左侧丢线判定从 `x <= start_point_x+2`(固定=8) 改为 `x <= left_bound_x + 3`（基于实际左边线位置偏移）
- 或者简单方案：提高阈值容差（从+2提高到+5）

## Architecture Design

```
修改前数据流（问题链）:
  WarpPerspective → zi_shi_ying_warp(跟踪参数不适配)
       → 边线数据有偏差(飘出/穿透)
            → detect_corners(绝对阈值误判)
            → detect_junction_type(固定检测框误判)
            → detect_lost(固定边界误判)
                 → road_type 错误 → control.c PID错误

修改后数据流:
  WarpPerspective → zi_shi_ying_warp(turn<4, clip+=, 边界严格)
       → 边线数据准确紧贴真实边界
            → detect_corners(比例+绝对双判断)
            → detect_junction_type(增加偏移过滤)
            → detect_lost(放宽容差)
                 → road_type 准确 → control.c PID正确
```

## Directory Structure

```
testest/
├── code/
│   ├── img_process.h          # [MODIFY] ZSY_CLIP_VALUE改正值, 新增比例阈值常量
│   └── img_process.c          # [MODIFY] 边线跟踪算法+角点检测+路口检测+丢线检测
```
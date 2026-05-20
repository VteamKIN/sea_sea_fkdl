# 项目清理与整理计划

## 目标
1. 删除所有未使用的变量、宏、枚举
2. 将仅内部使用的变量/函数改为 static，移除多余的 extern 声明
3. 为所有变量添加适量注释（用途说明，不含冗余信息）
4. 修复 `current_target_dir` 缺少显式定义的问题

---

## 一、删除未使用的变量（7个）

### img_process.c / img_process.h
| 变量 | 行号(.c) | 行号(.h) | 原因 |
|------|---------|---------|------|
| `left_bound_x` | :26 | :138 | 完全未读写 |
| `right_bound_x` | :27 | :139 | 完全未读写 |
| `left_corner_index` | :34 | :148 | 已废弃，完全未读写 |
| `right_corner_index` | :35 | :149 | 已废弃，完全未读写 |
| `curve_stable_count` | :44 | - | static，完全未读写 |
| `junction_hold_count` | :51 | - | static，完全未读写 |
| `hold_junction_type` | :52 | - | static，完全未读写 |

同时删除 img_process.c:215-216 的 `ipts0_num = left_edge_count; ipts1_num = right_edge_count;` 赋值语句。

### 删除仅内部赋值无外部读取的变量（2个）
| 变量 | 行号(.c) | 行号(.h) | 原因 |
|------|---------|---------|------|
| `ipts0_num` | :24 | :136 | 仅内部赋值，无外部读取 |
| `ipts1_num` | :25 | :137 | 仅内部赋值，无外部读取 |

---

## 二、删除未使用的宏（6个）

### img_process.h
| 宏 | 行号 | 原因 |
|----|------|------|
| `NEAR_BOUNDARY_MAX` | :39 | 已废弃角点检测参数 |
| `CORNER_EXTEND_MIN` | :40 | 已废弃角点检测参数 |
| `CORNER_EXTEND_MAX` | :41 | 已废弃角点检测参数 |
| `CORNER_RATIO_MIN` | :42 | 已废弃角点检测参数 |
| `CORNER_RATIO_MAX` | :43 | 已废弃角点检测参数 |
| `ROAD_GAP_THRESHOLD` | :36 | 完全未使用 |

---

## 三、删除未使用的枚举（2个）

### img_process.h
| 枚举 | 行号 | 原因 |
|------|------|------|
| `CornerType` | :58-63 | 已废弃，完全未使用 |
| `ImgFlag` | :77-81 | 完全未使用 |

---

## 四、变量改为 static + 移除 extern 声明（4个）

| 变量 | 当前 | 改为 | 删除 extern |
|------|------|------|------------|
| `Tx` | `float Tx` | `static float Tx` | img_process.h:177 |
| `Ty` | `float Ty` | `static float Ty` | img_process.h:177 |
| `error_left` | `vint16` | `static vint16` | control.h:39 |
| `error_right` | `vint16` | `static vint16` | control.h:40 |

---

## 五、修复缺失定义（1个）

| 变量 | 问题 | 修复 |
|------|------|------|
| `current_target_dir` | img_process.h:159 有 extern 声明，img_process.c:635 有赋值，但**无显式定义** | 在 img_process.c 变量定义区添加 `volatile int8 current_target_dir = 0;` |

---

## 六、添加变量注释

为 img_process.c 和 control.c 中所有保留的变量添加简洁注释，格式统一为行尾注释。

### img_process.c 变量注释规范
```c
volatile uint16 img_process_time = 0;       // 图像处理耗时(μs)
volatile uint8 left_edge_count = 0;         // 左边线点数
int16 left_edge[MAX_EDGE_POINTS][2];        // 左边线坐标 [i][0]=x [i][1]=y
```

### control.c 变量注释规范
```c
float kp = 25.0f;                           // PID比例系数
vint16 error_image = 0;                     // 图像偏差(像素)
vint16 output = 0;                          // PID输出(速度偏移量)
```

---

## 七、实施步骤

1. **img_process.h**: 删除废弃宏、枚举、extern声明
2. **img_process.c**: 删除未使用变量、改 Tx/Ty 为 static、添加 current_target_dir 定义、添加注释
3. **control.h**: 移除 error_left/error_right 的 extern 声明
4. **control.c**: 改 error_left/error_right 为 static、添加注释
5. **验证**: 确认编译无错误

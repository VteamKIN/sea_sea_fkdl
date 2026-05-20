# 待修复问题：choose 数组越界检查

## 问题描述

**位置：** `control.c:313` 和 `img_process.c:1078`

```c
int8 target_dir = choose[dir_count];  // 没有边界检查
int8 choose[20] = {0, -1, -1, -1, -1, -99, 0};
```

**风险：** 当 `dir_count >= 20` 时，会访问非法内存，导致未定义行为。

## 建议修复方案

在 `control.c` 的 `evaluate_state()` 函数中添加边界检查：

```c
// 3.2 检查是否需要转弯（结合方向数组和道路类型）
// 边界检查：防止数组越界
if (dir_count >= 20)
{
    switch_state(RUN_STOP);
    return;
}

int8 target_dir = choose[dir_count];  // 获取当前目标方向
```

## 优先级

中等风险，建议在正式运行前修复。

## 状态

待修复

---

**创建时间：** 2026年4月7日  
**创建者：** 22222

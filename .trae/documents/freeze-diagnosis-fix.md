# 程序卡死问题诊断与修复计划

## 问题现象
程序有时会卡死：ISR 中的控制部分正常运行（编码器读取 + PID + 电机输出），但不会对图像有反应，一直是初始速度在跑。

## 根因分析

### 数据流路径
```
摄像头 DMA 完成 → mt9v03x_finish_flag=1 (ISR/CPU0)
    ↓
CPU1 主循环: if(mt9v03x_finish_flag && !cpu1_img_ready_flag)
    → img_process()  [CPU1]
    → cpu1_img_ready_flag = 1
    ↓
CPU0 主循环: if(cpu1_img_ready_flag)
    → calc_error_image()  [CPU0主循环]
    → cpu1_img_ready_flag = 0
    ↓
CPU0 ISR(2ms): control_process()
    → calc_error_image()  [CPU0 ISR]  ← ⚠️ 也在ISR中调用
    → image_pid_out()
    → motor_control()
```

### 关键发现：calc_error_image() 被调用了两次！

1. **CPU0 主循环** `cpu0_main.c:37` 调用 `calc_error_image()`
2. **CPU0 ISR** `isr.c:54` → `control_process()` → `calc_error_image()`

这意味着 `calc_error_image()` 在 ISR 和主循环中**同时被调用**，存在严重的竞态条件：
- ISR 中的 `control_process()` 正在读写 `error_image`、`last_error`、`hist[]` 等变量
- 主循环中的 `calc_error_image()` 也在读写这些变量
- ISR 可以打断主循环，导致数据不一致

### 卡死的根本原因（按可能性排序）

#### 原因 1：CPU1 图像处理停止（最可能 ⭐⭐⭐⭐⭐）

CPU1 主循环的条件是 `mt9v03x_finish_flag && (0 == cpu1_img_ready_flag)`。

**死锁场景**：
1. CPU1 完成 `img_process()`，设置 `cpu1_img_ready_flag = 1`
2. CPU0 主循环检测到 `cpu1_img_ready_flag = 1`，进入 `if` 分支
3. CPU0 主循环调用 `calc_error_image()`（耗时较长，有排序等操作）
4. **在 `calc_error_image()` 执行期间**，CPU0 ISR 触发，ISR 中的 `control_process()` 也调用 `calc_error_image()`
5. ISR 返回后，CPU0 主循环继续执行，最终 `cpu1_img_ready_flag = 0`
6. 正常情况下这不会死锁...

**但真正的死锁场景是**：
- CPU0 主循环中 `cpu1_img_ready_flag = 0` 这行代码（第87行）前面有 `__dsync()`
- 如果 CPU0 主循环**从未进入** `if(cpu1_img_ready_flag)` 分支（例如因为 ISR 中的 `control_process()` 已经在处理图像数据，CPU0 主循环被 ISR 频繁打断），那么 `cpu1_img_ready_flag` 永远不会被清零
- CPU1 永远看到 `cpu1_img_ready_flag == 1`，永远不处理新帧
- **结果：CPU1 停止图像处理，CPU0 ISR 中的 `control_process()` 使用最后一帧的陈旧数据**

等等，这不对。CPU0 主循环是空闲的（大部分显示代码被注释），应该能快速清零 `cpu1_img_ready_flag`。

让我重新分析...

**更可能的场景**：CPU0 主循环确实会清零 `cpu1_img_ready_flag`，但清零后 CPU1 需要等下一帧 `mt9v03x_finish_flag`。如果 DMA 传输出错（`mt9v03x_dma_handler` 中检测到 `TransactionRequestLost`），`mt9v03x_finish_flag` 会被置 0，`mt9v03x_dma_init_flag` 被置 1，需要等下一次 VSYNC 中断重新初始化 DMA。如果 DMA 频繁出错，CPU1 可能长时间得不到新帧。

#### 原因 2：跨核变量缺少 volatile（⭐⭐⭐⭐⭐）

这是**最可能的根因**！

CPU1 写入的变量（`road_type`、`left_edge[]`、`right_edge[]`、`left_edge_count`、`right_edge_count`、`left_lost_count`、`right_lost_count`、`current_junction`、`junction_detected`、`dir_count`）**全部没有 volatile 修饰**。

CPU0 ISR 中的 `control_process()` 读取这些变量时，编译器可能将它们缓存到寄存器中，**永远不重新从内存读取**。一旦编译器做了这样的优化，CPU0 将永远使用初始值（`road_type = straight`、`left_edge_count = 0` 等），导致：
- `calc_error_image()` 中 `sample_count < 5`，`error_image` 持续衰减到 0
- `output` 持续为 0
- 电机以 `control_base_speed`（1500）恒速运行
- **完全符合用户描述的"一直是初始速度在跑"**

#### 原因 3：calc_error_image 双重调用竞态（⭐⭐⭐）

CPU0 主循环和 ISR 同时调用 `calc_error_image()`，ISR 可以打断主循环：
- 主循环正在写 `hist[]` 数组，ISR 打断后也写 `hist[]`
- 主循环正在写 `error_image`，ISR 打断后覆盖 `error_image`
- `last_error` 被两个上下文同时修改

这会导致 `error_image` 计算错误，但不应该导致"完全无反应"。

#### 原因 4：CPU1 img_process() 死循环（⭐⭐）

`findline_lefthand_adaptive` 和 `findline_righthand_adaptive` 有 `turn < 4` 限制，理论上不会死循环。但如果 `row_prefix` 数据异常（例如未初始化或越界），可能导致 `local_thres` 计算出极端值，使得跟踪器在边界反复转向但 `turn` 始终被重置为 0。

不过这种情况不太可能，因为 `turn` 在每次转向时递增，前进时重置为 0，4 次连续转向就会退出。

#### 原因 5：camera.c 中 image_send_seekffree 的干扰（⭐⭐）

`camera.c` 中的 `image_send_seekffree()` 函数在第 153 行清零 `mt9v03x_finish_flag`，但这个函数似乎不在主循环中被调用（cpu0_main.c 中被注释了）。如果它被其他地方调用，会与 CPU1 的清零操作冲突。

## 修复方案

### 修复 1（最高优先级）：为跨核共享变量添加 volatile
**目标文件**: `img_process.h`、`img_process.c`

所有在 CPU1 写入、CPU0 读取的变量必须添加 `volatile`：
- `volatile RUN_Dir road_type`
- `volatile int16 left_edge[][2]`、`volatile int16 right_edge[][2]`
- `volatile uint8 left_edge_count`、`volatile uint8 right_edge_count`
- `volatile uint8 left_lost_count`、`volatile uint8 right_lost_count`
- `volatile enum JunctionType current_junction`
- `volatile uint8 junction_detected`
- `volatile uint8 dir_count`
- `volatile int8 choose[]`
- `volatile uint8 road_width_avg`
- `volatile uint16 img_process_time`
- `volatile uint8 global_warp_thres`

### 修复 2（高优先级）：移除 CPU0 主循环中的 calc_error_image() 调用
**目标文件**: `cpu0_main.c`

CPU0 主循环第 37 行的 `calc_error_image()` 调用是多余的且危险的：
- ISR 中的 `control_process()` 已经调用了 `calc_error_image()`
- 主循环中的调用与 ISR 存在竞态条件
- 应该移除主循环中的 `calc_error_image()` 调用

### 修复 3（高优先级）：CPU0 主循环清零 cpu1_img_ready_flag 时确保内存屏障
**目标文件**: `cpu0_main.c`

当前代码已有 `__dsync()`，但需确认位置正确（在写 flag 之前）。

### 修复 4（中优先级）：为 encoder 共享变量添加 volatile
**目标文件**: `encoder.c`、`encoder.h`

`encoder_data_l`、`encoder_data_r`、`encoder_conversion_l`、`encoder_conversion_r` 在 ISR 中写入，可能被主循环读取，需添加 volatile。

### 修复 5（中优先级）：添加首帧图像就绪保护
**目标文件**: `control.c`

在 `control_process()` 入口添加检查，确保至少处理过一帧图像后才执行控制：
```c
static uint8 first_frame_ready = 0;
if (!first_frame_ready) {
    if (img_process_time > 0) first_frame_ready = 1;
    else { motor_control(0, 0); return; }
}
```

### 修复 6（低优先级）：添加 CPU1 图像处理超时检测
**目标文件**: `cpu1_main.c`

添加调试计数器，检测 CPU1 是否长时间未处理新帧：
```c
static uint32 idle_count = 0;
if(mt9v03x_finish_flag && (0 == cpu1_img_ready_flag)) {
    idle_count = 0;
    img_process();
    ...
} else {
    idle_count++;
    // 如果 idle_count 超过阈值，说明可能卡死
}
```

## 实施顺序

1. **修复 1** - 添加 volatile（最关键，最可能是卡死根因）
2. **修复 2** - 移除主循环中的 calc_error_image()
3. **修复 3** - 确认内存屏障
4. **修复 4** - encoder volatile
5. **修复 5** - 首帧保护
6. **修复 6** - 超时检测（可选调试辅助）

## 验证方法

1. 编译后烧录，观察是否还会卡死
2. 在 CPU0 主循环中添加 `img_process_time` 的 TFT 显示，确认 CPU1 是否持续更新
3. 在 `control_process()` 中添加 `road_type` 和 `error_image` 的串口输出，确认值是否在变化

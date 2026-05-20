# Bug 分析报告：取消注释 L_control/R_control 后 assert error

## 1. 问题描述

| 项目 | 内容 |
|------|------|
| 现象 | 取消注释 `isr.c:53` 的 `L_control(700)` 和 `isr.c:59` 的 `R_control(700)` 后，屏幕报错 |
| 错误信息 | `assert error file:zf_device\zf_device_tft180.c line:550` |
| 正常情况 | 注释掉这两行后，屏幕正常显示图像 |
| 编译信息 | ROM: 0x12f2d (77613) = code: 0xf984 (63876) + romdata: 0x35a9 (13737) |
| 编译信息 | RAM: 0x1b66f (112239) = data + bss |
| 芯片内存 | 约 120KB |

---

## 2. 相关代码位置

### 触发位置

```c
// isr.c:43-60  PIT 定时器中断（每2ms触发一次）
IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU60_CH0);
    encoder_data_l = encoder_get_count(TIM2_ENCODER);
    encoder_clear_count(TIM2_ENCODER);
    encoder_conversion_l = encoder_data_l * 38;
    L_PID.ActualSpeed = _wangyi_(encoder_conversion_l);
    //L_control(700);          // ← 取消注释后出问题
    encoder_data_r = -encoder_get_count(TIM4_ENCODER);
    encoder_clear_count(TIM4_ENCODER);
    encoder_conversion_r = encoder_data_r * 38;
    R_PID.ActualSpeed = _wangyi_(encoder_conversion_r);
    //R_control(700);          // ← 取消注释后出问题
}
```

### 报错位置

```c
// zf_device_tft180.c:545-550
void tft180_show_string(uint16 x, uint16 y, const char dat[])
{
    zf_assert(x < tft180_width_max);
    zf_assert(y < tft180_height_max);   // ← 此行触发 assert
    // ...
}
```

### 调用链（主循环中触发）

```
cpu0_main.c:79  tft_show_warp_with_boundary()
  → tft.c:55    tft180_show_gray_image(...)        // 显示逆透视图像
  → tft.c:73    tft180_draw_point(...)              // 绘制边界点
  → cpu0_main.c:82  tft180_show_string(81, 0, "error:")  // ← 崩溃点
```

---

## 3. 硬件资源分析

### 3.1 链接脚本 RAM 配置（Lcf_Tasking_Tricore_Tc.lsl）

| 区域 | 大小 | 用途 |
|------|------|------|
| DSPR0 (CPU0 RAM) | **72KB** | CPU0 全局变量 + 栈 |
| DSPR1 (CPU1 RAM) | **120KB** | CPU1 全局变量 + 栈 |
| USTACK0 (CPU0 用户栈) | **2KB** | CPU0 主循环栈帧 |
| ISTACK0 (CPU0 中断栈) | **1KB** | CPU0 中断处理栈帧 |
| USTACK1 (CPU1 用户栈) | **2KB** | CPU1 主循环栈帧 |
| ISTACK1 (CPU1 中断栈) | **1KB** | CPU1 中断处理栈帧 |
| CSA0 (CPU0 上下文) | **8KB** | CPU0 中断上下文保存 |
| CSA1 (CPU1 上下文) | **8KB** | CPU1 中断上下文保存 |
| LCF_DEFAULT_HOST | **CPU1** | 未指定 section 的全局变量默认放 CPU1 RAM |

### 3.2 RAM 预算分析

**总 RAM 使用：112,239 bytes (109.6KB)**

由于 `LCF_DEFAULT_HOST = LCF_CPU1`，未指定 `#pragma section` 的全局变量（包括库函数的变量）全部放在 CPU1 的 120KB RAM 中。

CPU1 可用于全局变量的空间：
```
120KB (总) - 8KB (CSA1) - 2KB (USTACK1) - 1KB (ISTACK1) - 1KB (对齐) = 约 108KB
```

> **结论：112KB 全局变量已超出 CPU1 可用空间（约108KB），RAM 已溢出！**

### 3.3 大内存消耗变量清单

| 变量 | 大小 | 所在文件 | section |
|------|------|----------|---------|
| `imgGray[128][160]` | 20,480 B | control_process.c | cpu1_dsram |
| `imgOSTU[128][160]` | 20,480 B | control_process.c | cpu1_dsram |
| `warp_image[128][160]` | 20,480 B | control_process.c | cpu1_dsram |
| `histogram[256]` (uint16_t) | 512 B | control_process.c | cpu1_dsram |
| `mt9v03x_image[120][188]` | 22,560 B | 库文件 | 默认(CPU1) |
| `L_line[100][2]` (float) | 800 B | control_process.c (栈上) | 栈 |
| `R_line[100][2]` (float) | 800 B | control_process.c (栈上) | 栈 |
| **小计（仅图像相关）** | **约 86KB** | | |

---

## 4. 根因分析

### 4.1 根因一（主要）：CPU1 RAM 溢出，全局变量被栈覆盖

**机制说明：**

```
CPU1 DSPR 内存布局（从低到高）：

┌──────────────────────┐ 0x60000000
│                      │
│   全局变量区域        │  ← tft180_width_max, tft180_height_max 等在此处
│   （112KB 使用中）    │
│                      │
├──────────────────────┤ ~0x6001B000
│                      │
│   HEAP1 (2KB)        │
├──────────────────────┤
│   USTACK1 (2KB)      │  ← 栈向下增长，可能覆盖上方全局变量
├──────────────────────┤
│   padding            │
├──────────────────────┤
│   ISTACK1 (1KB)      │
├──────────────────────┤
│   padding            │
├──────────────────────┤
│   CSA1 (8KB)         │
└──────────────────────┘ 0x6001E000
```

当 `L_control`/`R_control` 被取消注释后：
1. 编译器链接了这些函数及其调用的 `Motor_Left`/`Motor_Right`/`constrain` 等代码
2. 增加了少量 ROM 和 RAM 占用（函数内的局部变量、增量PID的 `int32 delta_Output` 等）
3. 更关键的是，**中断频率执行使得栈的使用更加活跃**
4. `get_BLY()` 函数在 CPU1 上执行，内部有 `float L_line[100][2]` 和 `float R_line[100][2]` 共 **1,600 字节** 的栈上局部变量
5. 栈增长时覆盖了相邻的全局变量区域，`tft180_height_max` 被破坏为异常值
6. 主循环调用 `tft180_show_string(81, 0, "error:")` 时，`y=0` 不再满足 `y < tft180_height_max`（因为 `tft180_height_max` 已被破坏为 0 或垃圾值），触发 assert

### 4.2 根因二（次要）：CPU0 中断打断屏幕 SPI 刷新

PIT 中断优先级 30，在主循环 `tft_show_warp_with_boundary()` 执行期间可以随时打断：

```
主循环 (CPU0)                    PIT 中断 (CPU0)
  │                                │
  ├─ tft180_set_region(...)        │
  │  (设置SPI显示区域)              │
  │                                ├─ L_control(700)
  │                                │  └─ Motor_Left(...)
  │                                │     └─ pwm_set_duty(...)
  ├─ tft180_write_16bit_data_array │  (PWM配置，可能涉及ATOM寄存器)
  │  (SPI发送像素数据)              │
  │  ← 中断返回                    │
  │                                │
  ├─ tft180_show_string(81,0,...)  │
  │  └─ zf_assert(y < height_max)  │  ← 如果 height_max 被破坏，这里崩溃
```

虽然 PWM 和 SPI 不是同一外设，但中断嵌套增加了栈深度，加剧了栈溢出的风险。

### 4.3 为什么注释掉就不出问题

| 对比项 | 注释掉 L_control/R_control | 取消注释 |
|--------|---------------------------|----------|
| RAM 占用 | 略少（编译器可能优化掉未引用函数） | 略多（必须链接 Motor_Left/Right 等） |
| 中断执行时间 | 短（仅读取编码器+滤波） | 长（增加PID计算+PWM输出） |
| 中断栈深度 | 浅 | 更深（多一层函数调用） |
| 栈溢出风险 | 刚好在边界内 | 超出边界，覆盖全局变量 |

---

## 5. 排查与修复计划

### 步骤 1：验证全局变量是否被破坏（最快定位）

在 `cpu0_main.c` 的主循环中，调用 `tft180_show_string` 之前添加调试输出：

```c
// cpu0_main.c — 在 tft_show_warp_with_boundary() 之前添加
if(cpu1_img_ready_flag)
{
    // 调试：打印屏幕尺寸变量，验证是否被破坏
    uart_printf("w=%d h=%d error=%d\r\n", tft180_width_max, tft180_height_max, error);
    
    calc_error();
    tft_show_warp_with_boundary();
    tft180_show_string(81, 0, "error:");
    // ...
}
```

**预期结果**：
- 正常时应输出 `w=160 h=128`
- 如果被破坏，会输出异常值（如 0、随机大数等）

### 步骤 2：减少 RAM 占用（治本）

#### 2.1 合并图像缓冲区（节省约 20KB）

`imgGray` 和 `imgOSTU` 不需要同时存在，可以在二值化时直接覆盖：

```c
// control_process.c — 修改前
uint8 imgGray[IMG_H][IMG_W];    // 20KB
uint8 imgOSTU[IMG_H][IMG_W];    // 20KB

// control_process.c — 修改后（共用同一块内存）
uint8 img_process_buf[IMG_H][IMG_W];  // 20KB（替代上述两个数组）
#define imgGray   img_process_buf
#define imgOSTU   img_process_buf
```

> **注意**：需要检查所有同时使用 `imgGray` 和 `imgOSTU` 的地方，确保不会出现数据冲突。

#### 2.2 `warp_image` 原地操作（节省约 20KB）

逆透视图像仅在显示时使用，可以在显示时实时计算，或复用 `imgOSTU` 的内存：

```c
// 修改方案：WarpPerspective 输出到 imgOSTU 缓冲区（在 CPU1 处理完后）
// 或在 tft_show_warp_with_boundary 中实时计算
```

#### 2.3 缩小 histogram 类型（节省 256B）

```c
// control_process.c — 修改前
uint16_t histogram[256];    // 512 bytes

// 修改后
uint8_t histogram[256];     // 256 bytes（单帧最大像素数不会超过255*160=40800）
```

#### 2.4 将大数组从栈移到全局（减少栈溢出风险）

```c
// control_process.c — get_BLY() 中
// 修改前：栈上分配 1,600 字节
float L_line[100][2];    // 800 bytes（栈上）
float R_line[100][2];    // 800 bytes（栈上）

// 修改后：全局分配
static float L_line[100][2];
static float R_line[100][2];
```

#### 2.5 double 改 float（节省少量 RAM）

```c
// control_process.c — 修改前
static double Mat1[3][3] = {...};   // 72 bytes
static double Mat2[3][3] = {...};   // 72 bytes
double Tx = 0;                      // 8 bytes
double Ty = 0;                      // 8 bytes

// 修改后
static float Mat1[3][3] = {...};    // 36 bytes
static float Mat2[3][3] = {...};    // 36 bytes
float Tx = 0;                       // 4 bytes
float Ty = 0;                       // 4 bytes
```

### 步骤 3：调整内存分区（治本）

将部分大数组从 CPU1 RAM 移到 CPU0 RAM：

```c
// cpu0_main.c 中已有 #pragma section all "cpu0_dsram"
// 可以在此处定义需要放在 CPU0 的大数组
#pragma section all "cpu0_dsram"
uint8 image_buffer[128][160];   // 20KB 放到 CPU0 的 72KB RAM 中
#pragma section all restore
```

同时确保 `tft180_width_max` 和 `tft180_height_max` 等关键全局变量不会被覆盖。

### 步骤 4：增大 CPU0 栈空间

修改链接脚本 `Lcf_Tasking_Tricore_Tc.lsl`：

```c
// 修改前
#define LCF_USTACK0_SIZE    2k
#define LCF_ISTACK0_SIZE    1k

// 修改后
#define LCF_USTACK0_SIZE    4k    // 增大用户栈
#define LCF_ISTACK0_SIZE    2k    // 增大中断栈
```

### 步骤 5：降低中断频率（临时验证）

```c
// encoder.c — 修改前
#define encoder_time    2    // 2ms 中断周期

// 临时改为
#define encoder_time    10   // 10ms 中断周期（降低中断压力）
```

如果问题消失，则进一步确认是栈/RAM 不足导致的。

---

## 6. 推荐修复优先级

| 优先级 | 修复项 | 预期效果 | 难度 |
|--------|--------|----------|------|
| **P0** | 步骤 1：验证全局变量被破坏 | 确认根因 | 低 |
| **P0** | 步骤 2.1：合并 imgGray/imgOSTU | 节省 20KB RAM | 中 |
| **P0** | 步骤 2.4：L_line/R_line 移出栈 | 防止栈溢出 | 低 |
| **P1** | 步骤 2.2：warp_image 复用 | 节省 20KB RAM | 中 |
| **P1** | 步骤 4：增大栈空间 | 防止栈溢出 | 低 |
| **P2** | 步骤 2.3+2.5：小优化 | 节省约 300B | 低 |
| **P2** | 步骤 3：调整内存分区 | 均衡 RAM 使用 | 中 |
| **验证** | 步骤 5：降低中断频率 | 快速验证 | 低 |

---

## 7. 修复后验证清单

- [ ] 编译后确认 RAM 使用量下降（目标 < 100KB）
- [ ] 恢复 `L_control(700)` 和 `R_control(700)` 的注释
- [ ] 烧录运行，确认屏幕正常显示
- [ ] 观察串口调试输出，确认 `tft180_width_max=160, tft180_height_max=128`
- [ ] 长时间运行测试（确认不会偶发栈溢出）

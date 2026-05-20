# 内存溢出风险与图像处理→控制链路问题排查 Spec

## Why
TC264D 双核嵌入式系统中，CPU1 DSRAM 占用率已接近 80%，新增的积分图（40.5KB）进一步加剧了内存压力。同时，图像处理（CPU1）到控制（CPU0 中断）的数据链路存在多核竞态、数组越界、整数溢出等多类风险，可能导致车辆失控或系统崩溃。

## What Changes
- 修复 CPU1 DSRAM 内存溢出风险（积分图过大 + 默认段变量叠加）
- 修复跨核共享变量的竞态条件
- 修复数组越界、整数溢出、除零等运行时风险
- 修复 volatile 缺失导致的编译器优化风险
- 修复控制链路中的逻辑缺陷

## Impact
- Affected specs: optimize-img-process（积分图新增导致内存压力）
- Affected code: `img_process.c/h`、`control.c/h`、`encoder.c/h`、`imu.c/h`、`motor.c/h`、`camera.c/h`、`cpu0_main.c`、`cpu1_main.c`、`isr.c`

---

## ADDED Requirements

### Requirement: CPU1 DSRAM 内存安全
系统 SHALL 确保 CPU1 DSRAM 总占用不超过 120KB 上限，并留有合理余量。

#### Scenario: 积分图导致内存溢出
- **WHEN** `integral_image[81][125]` (uint32, 40,500字节) + `mt9v03x_image[188][120]` (22,560字节) + `image_copy[188][120]` (22,560字节) + `warp_lut[80][124]` (19,840字节) + `warp_image[80][124]` (9,920字节) + 其他变量 同时存在于 CPU1 DSRAM
- **THEN** 总占用约 117KB，接近 120KB 上限，链接时可能溢出或运行时栈溢出

#### Scenario: 积分图可降级为 uint16
- **WHEN** warp_image 单行像素和最大为 124×255 = 31,620 < 65,535
- **THEN** 可将 `uint32 integral_image` 改为 `uint16 integral_image`，节省 20,250 字节

### Requirement: 跨核共享变量同步安全
系统 SHALL 确保所有跨核共享变量使用适当的同步机制（volatile + 内存屏障）。

#### Scenario: CPU1 写入图像处理结果，CPU0 读取
- **WHEN** CPU1 的 `img_process()` 写入 `road_type`、`left_edge[]`、`right_edge[]`、`error_image`、`output` 等变量
- **AND** CPU0 的 `control_process()` 在 PIT 中断中读取这些变量
- **THEN** 存在竞态条件：CPU0 可能在 CPU1 写入一半时读取，导致数据不一致

#### Scenario: cpu1_img_ready_flag 同步
- **WHEN** CPU1 设置 `cpu1_img_ready_flag = 1` 后 CPU0 读取
- **THEN** 当前使用 `__dsync()` 屏障是正确的，但 CPU0 侧清零时也需确保屏障

### Requirement: 数组越界防护
系统 SHALL 确保所有数组访问在合法索引范围内。

#### Scenario: choose 数组越界
- **WHEN** `dir_count` 递增超过 `road_num`(20)
- **THEN** `choose[dir_count]` 将越界访问，虽然 `control_process()` 有 `dir_count >= road_num` 检查，但 `detect_road_type()` 也有同样的检查，两处检查需保持一致

#### Scenario: sample_x 数组越界
- **WHEN** `calc_error_image()` 中 `sample_count` 超过 200（`sample_x[200]` 数组大小）
- **THEN** 如果左右边线点数都接近 200，采样区域内的点数可能超过 200，导致栈溢出

#### Scenario: left_edge/right_edge 写入越界
- **WHEN** 边线跟踪步数达到 `MAX_EDGE_POINTS`(200) 时循环终止
- **THEN** 当前实现安全，但 `left_edge_count`/`right_edge_count` 为 uint8，若超过 255 会回绕

#### Scenario: encoder_conversion 溢出
- **WHEN** `encoder_data_l * 38` 或 `encoder_data_r * 38` 超过 int16 范围
- **THEN** `encoder_data_l` 最大约 ±32767，乘以 38 后溢出 int16，但 `encoder_conversion_l` 声明为 int16

### Requirement: 整数溢出防护
系统 SHALL 确保算术运算不产生未定义的整数溢出。

#### Scenario: PID 增量计算溢出
- **WHEN** `L_PID.Kp * (L_PID.Err - L_PID.ErrLast)` 中 Kp 为 float，Err 为 int16
- **THEN** float × int16 结果为 float，不会溢出（已修复为 float 中间变量）

#### Scenario: 电机速度计算溢出
- **WHEN** `control_base_speed + output` 或 `control_base_speed - output` 超过 int16 范围
- **THEN** `control_base_speed = 1500`，`output` 限幅 ±8000，结果范围 [-6500, 9500]，在 int16 范围内

#### Scenario: encoder_conversion 乘法溢出
- **WHEN** `encoder_data_l * 38`，encoder_data_l 为 int16（范围 ±32767）
- **THEN** 32767 × 38 = 1,245,146，超出 int16 范围（±32767），但 `encoder_conversion_l` 为 int16，会发生截断

### Requirement: 除零防护
系统 SHALL 确保所有除法运算的分母不为零。

#### Scenario: 边线跟踪中的块大小
- **WHEN** `block_size = ZSY_BLOCK_SIZE = 7`，`block_size * block_size = 49`
- **THEN** 常量除法，安全

#### Scenario: base_count 为零
- **WHEN** `detect_junction_type()` 中 `base_count` 可能为 0
- **THEN** 已有 `if (base_count > 0)` 保护，安全

#### Scenario: sample_count 为零
- **WHEN** `calc_error_image()` 中 `sample_count < 5` 时提前返回
- **THEN** `sample_x[sample_count / 2]` 不会除零，安全

### Requirement: volatile 正确使用
系统 SHALL 对所有在中断/跨核上下文中共享的变量使用 volatile 限定符。

#### Scenario: 图像处理结果变量缺少 volatile
- **WHEN** `road_type`、`left_edge[]`、`right_edge[]`、`left_edge_count`、`right_edge_count`、`left_lost_count`、`right_lost_count`、`current_junction`、`junction_detected`、`dir_count` 在 CPU1 写入、CPU0 读取
- **THEN** 这些变量均未声明为 volatile，编译器优化可能导致 CPU0 读取到旧值

#### Scenario: control.c 中的共享变量
- **WHEN** `car_running`、`error_image`、`output` 在 ISR 中使用
- **THEN** `car_running` 声明为 `vint8`（逐飞库的 volatile int16），`error_image` 和 `output` 为 `vint16`，已正确使用 volatile

#### Scenario: encoder.c 中的共享变量
- **WHEN** `encoder_data_l`、`encoder_data_r` 在 ISR 中写入、主循环读取
- **THEN** 未声明为 volatile，可能导致主循环读到缓存旧值

### Requirement: 栈溢出防护
系统 SHALL 确保函数局部变量不会导致栈溢出。

#### Scenario: calc_error_image 栈上大数组
- **WHEN** `int16 sample_x[200]` 在栈上分配 400 字节
- **THEN** 在 PIT 中断中调用 `control_process()` → `calc_error_image()`，中断栈空间有限，400 字节可能接近栈上限

#### Scenario: 中断嵌套加剧栈压力
- **WHEN** ISR 中调用 `interrupt_global_enable(0)` 开启中断嵌套
- **THEN** 多层中断嵌套时栈使用叠加，可能溢出

### Requirement: 控制链路逻辑完整性
系统 SHALL 确保从图像处理到电机控制的完整链路无逻辑缺陷。

#### Scenario: CPU0 中断直接调用 control_process 但未等图像就绪
- **WHEN** PIT 中断每 2ms 触发 `control_process()`，不检查图像是否已就绪
- **THEN** 第一帧图像到来之前，`road_type`、`left_edge[]` 等变量为零/空，`calc_error_image()` 可能使用未初始化的数据

#### Scenario: motor_control 函数签名不一致
- **WHEN** `encoder.c` 中的 `motor_control(int16, int16)` 调用 `motor.c` 中的 `motor_set(int, int)`
- **THEN** `motor_left(int speed)` 和 `motor_right(int speed)` 的参数类型为 int，而调用者传入 int16，隐式提升安全但不够规范

#### Scenario: PWM 占空比未限幅
- **WHEN** `motor_left(speed)` 中 `pwm_set_duty(L_CH, speed)` 直接使用 speed 作为占空比
- **THEN** 如果 speed > 10000（PWM 周期），行为未定义；`speed_max = 8000` 但未在 motor 层强制限幅

#### Scenario: 编码器滤波函数非线程安全
- **WHEN** `_wangyi_()` 函数使用 `static` 局部变量 `last_raw1`、`last_raw2` 和 `EncoderFilter f`
- **THEN** 如果从多个上下文调用（虽然当前只在 ISR 中），static 变量会被破坏

#### Scenario: CPU1 看门狗已关闭
- **WHEN** `core1_main()` 中调用 `disable_Watchdog()`
- **THEN** CPU1 无看门狗保护，若 `img_process()` 死循环，CPU1 将永久挂起

---

## MODIFIED Requirements

### Requirement: 积分图内存优化（修改自 optimize-img-process spec）
将 `uint32 integral_image[81][125]` 改为 `uint16 integral_image[81][125]`，因为 warp_image 单行最大像素和为 124×255 = 31,620 < 65,535，uint16 足够存储。节省 20,250 字节（约 19.8KB）。

### Requirement: 跨核变量添加 volatile（修改自 optimize-img-process spec）
所有在 CPU1 写入、CPU0 读取的图像处理结果变量需添加 volatile 限定符。

## REMOVED Requirements
无。

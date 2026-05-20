# Tasks

## 一、内存溢出风险修复

- [ ] Task 1: 积分图类型降级 uint32→uint16，节省 20,250 字节
  - [ ] SubTask 1.1: 修改 img_process.c 中 `integral_image` 定义从 `uint32` 改为 `uint16`
  - [ ] SubTask 1.2: 修改 img_process.h 中 `integral_image` 的 extern 声明
  - [ ] SubTask 1.3: 修改 `build_integral_image()` 中的中间变量 `row_sum` 和 `int_row` 类型
  - [ ] SubTask 1.4: 修改 `findline_lefthand/righthand_adaptive()` 中 `area_sum` 变量类型
  - [ ] SubTask 1.5: 验证 warp_image 单行像素和最大值不超过 65535

- [ ] Task 2: 评估 image_copy 是否可移除或迁移至 PSRAM
  - [ ] SubTask 2.1: 确认 image_copy 是否在当前代码中被使用（camera.c 中的 image_send_seekffree 似乎未在主循环调用）
  - [ ] SubTask 2.2: 若未使用，将 image_copy 及相关边界数组用条件编译包裹或移除
  - [ ] SubTask 2.3: 若需保留，将 image_copy 迁移至 cpu1_psram 段

- [ ] Task 3: 评估 camera.c 边界数组是否可移除
  - [ ] SubTask 3.1: 确认 xy_x1_boundary 等 6 个数组是否在当前代码中使用
  - [ ] SubTask 3.2: 若未使用，用条件编译包裹或移除，节省 ~1.7KB

## 二、跨核竞态条件修复

- [ ] Task 4: 为跨核共享变量添加 volatile
  - [ ] SubTask 4.1: 在 img_process.h 中为以下 extern 变量添加 volatile：`road_type`、`left_edge`、`right_edge`、`left_edge_count`、`right_edge_count`、`left_lost_count`、`right_lost_count`、`current_junction`、`junction_detected`、`dir_count`、`choose`、`road_width_avg`
  - [ ] SubTask 4.2: 在 img_process.c 中对应的变量定义添加 volatile
  - [ ] SubTask 4.3: 在 encoder.c 中为 `encoder_data_l`、`encoder_data_r`、`encoder_conversion_l`、`encoder_conversion_r` 添加 volatile

- [ ] Task 5: CPU0 侧清零 cpu1_img_ready_flag 时添加内存屏障
  - [ ] SubTask 5.1: 在 cpu0_main.c 中 `cpu1_img_ready_flag = 0` 前添加 `__dsync()`

## 三、数组越界与整数溢出修复

- [ ] Task 6: 修复 encoder_conversion 乘法溢出
  - [ ] SubTask 6.1: 将 `encoder_conversion_l = encoder_data_l * 38` 改为使用 int32 中间变量
  - [ ] SubTask 6.2: 将 `encoder_conversion_r = encoder_data_r * 38` 同样修复
  - [ ] SubTask 6.3: 将 `encoder_conversion_l/r` 变量类型从 int16 改为 int32

- [ ] Task 7: 修复 calc_error_image 中 sample_x 数组越界风险
  - [ ] SubTask 7.1: 在 sample_x 写入循环中添加 `sample_count < 200` 的边界检查

- [ ] Task 8: 修复 motor_left/motor_right PWM 占空比未限幅
  - [ ] SubTask 8.1: 在 motor_left/motor_right 函数入口添加占空比限幅（0~10000）

## 四、控制链路逻辑修复

- [ ] Task 9: 添加首帧图像就绪检查
  - [ ] SubTask 9.1: 在 control_process() 入口添加 `img_process_time > 0` 检查，首帧图像未就绪时跳过控制

- [ ] Task 10: 统一 motor_control 函数签名
  - [ ] SubTask 10.1: 将 motor_left/motor_right 参数类型从 int 改为 int16
  - [ ] SubTask 10.2: 将 motor_set 参数类型从 int 改为 int16

## 五、验证

- [ ] Task 11: 编译验证
  - [ ] SubTask 11.1: 确保所有修改无编译错误
  - [ ] SubTask 11.2: 检查 CPU1 DSRAM 占用是否降至安全范围（< 100KB）

- [ ] Task 12: 功能验证
  - [ ] SubTask 12.1: 验证积分图 uint16 精度足够
  - [ ] SubTask 12.2: 验证边线跟踪结果正确
  - [ ] SubTask 12.3: 验证控制链路首帧安全

# Task Dependencies
- [Task 1] 积分图降级是最高优先级，直接影响内存安全
- [Task 2, 3] 内存优化可与 Task 1 并行
- [Task 4] volatile 修改独立于其他任务
- [Task 6] encoder 溢出修复独立
- [Task 7, 8] 数组/PWM 限幅独立
- [Task 9, 10] 控制链路修复独立
- [Task 11, 12] 依赖所有前置任务完成

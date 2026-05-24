# 21届疯狂电路

## 主要功能

- 摄像头图像采集与道路/车道检测（`code/camera.c`，`code/img_process.c`）
- 控制算法：PID 与速度/方向控制、寻线策略（`code/control.c`，`code/seekfree.c`）
- 传感器处理：编码器、IMU 读取与卡尔曼滤波（`code/encoder.c`，`code/imu.c`，`code/kalman.c`）
- 电机驱动与 PWM 输出（`code/motor.c`）
- TFT 菜单与显示（`code/tft.c`，`code/menu.c`）
- 无线通信/调试接口（`code/wireless_customer.c`）

## 目录结构

- `code/`：主源码（摄像头、控制、滤波、驱动等模块）
- `Debug/`：构建输出与调试文件
- `libraries/`：外部或共享库（如有）
- `user/`：目标板/用户相关配置与脚本

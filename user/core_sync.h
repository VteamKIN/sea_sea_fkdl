#ifndef USER_CORE_SYNC_H_
#define USER_CORE_SYNC_H_

#include "zf_common_headfile.h"

// CPU1 完成一帧"图像处理结果"的发布标志：
// - CPU1: 处理并写完共享数据后置 1
// - CPU0: 显示完读取后清 0
extern volatile vuint8 cpu1_img_ready_flag;

// 负压驱动就绪后才允许控制轮子（CPU0 置 1，ISR 中判断）
extern volatile vuint8 control_enable_flag;

#endif /* USER_CORE_SYNC_H_ */

/*
 * seekfree.h
 *
 *  Created on: 2026年3月11日
 *      Author: aaa
 */

#ifndef CODE_SEEKFREE_H_
#define CODE_SEEKFREE_H_

// 发送图像处理结果到逐飞助手的模式
typedef enum
{
    SEND_MODE_IMAGE_ONLY        = 0x01,   // 仅发送灰度图像
    SEND_MODE_IMAGE_WITH_BOUND  = 0x02,   // 灰度图像 + 左右边界 + 中线
    SEND_MODE_ALL_FEATURES      = 0x04,   // 在上面的基础上，再通过示波器通道发送道路特征
} IMG_SEND_MODE;

// 发送当前帧的灰度图像 / 边线 / 道路类型到上位机
void send_img_result_to_pc(uint8 mode);



#endif /* CODE_SEEKFREE_H_ */

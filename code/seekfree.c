/*
 * seekfree.c
 *
 *  Created on: 2026年3月11日
 *      Author: aaa
 */


#include "zf_common_headfile.h"

/*******************************************************************************
 * 发送当前帧的图像处理结果到逐飞助手上位机
 * 参数 mode - 发送模式（灰度图/带边线/完整特征）
 *******************************************************************************/
void send_img_result_to_pc(uint8 mode)
{
    // 1. 配置要发送的图像：逆透视灰度图像
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, warp_image_ptr, WARP_IMAGE_W, WARP_IMAGE_H);

    // 2. 根据 mode 决定是否发送边线
    if (mode & (SEND_MODE_IMAGE_WITH_BOUND | SEND_MODE_ALL_FEATURES))
    {
        static uint8 left_x[WARP_IMAGE_H];
        static uint8 right_x[WARP_IMAGE_H];
        static uint8 center_x[WARP_IMAGE_H];

        // 从原始边线点构建逐行边界
        for (int y = 0; y < WARP_IMAGE_H; y++)
        {
            left_x[y] = 0;
            right_x[y] = 0;
            center_x[y] = 0;
        }

        // 左边线：每行取最后一个点（最上方）
        for (int i = 0; i < left_edge_count; i++)
        {
            int y = left_edge[i][1];
            int x = left_edge[i][0];
            if (y >= 0 && y < WARP_IMAGE_H && x >= 0 && x < WARP_IMAGE_W)
                left_x[y] = (uint8)x;
        }

        // 右边线：每行取最后一个点（最上方）
        for (int i = 0; i < right_edge_count; i++)
        {
            int y = right_edge[i][1];
            int x = right_edge[i][0];
            if (y >= 0 && y < WARP_IMAGE_H && x >= 0 && x < WARP_IMAGE_W)
                right_x[y] = (uint8)x;
        }

        // 中线：取左右边线均值
        for (int y = 0; y < WARP_IMAGE_H; y++)
        {
            if (left_x[y] > 0 && right_x[y] > 0)
                center_x[y] = (uint8)((left_x[y] + right_x[y]) / 2);
        }

        // 发送三条边界线：左边界 / 右边界 / 中线
        seekfree_assistant_camera_boundary_config(X_BOUNDARY,
                                                  WARP_IMAGE_H,
                                                  left_x,
                                                  right_x,
                                                  center_x,
                                                  NULL,
                                                  NULL,
                                                  NULL);
    }
    else
    {
        // 不带边界
        seekfree_assistant_camera_boundary_config(NO_BOUNDARY,
                                                  0,
                                                  NULL, NULL, NULL,
                                                  NULL, NULL, NULL);
    }

    // 3. 发送图像 + 边线数据
    seekfree_assistant_camera_send();

    // 4. 发送道路特征（当前偏差、道路类型、dir_count、转弯方向）
    if (mode & SEND_MODE_ALL_FEATURES)
    {
        // 通道数量：4个
        seekfree_assistant_oscilloscope_data.channel_num = 4;

        // ch1: 当前偏差
        seekfree_assistant_oscilloscope_data.data[0] = (float)error_image;

        // ch2: 当前运行方向
        seekfree_assistant_oscilloscope_data.data[1] = (float)road_type;

        // ch3: 当前dir_count
        seekfree_assistant_oscilloscope_data.data[2] = (float)dir_count;

        // ch4: 运行状态（0:停车 1:运行）
        seekfree_assistant_oscilloscope_data.data[3] = (float)car_running;

        seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
    }
}

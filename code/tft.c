/*
 * tft.c
 *
 *  Created on: 2026年1月15日
 *      Author: aaa
 */
#include "zf_common_headfile.h"
void tft_init(void)
{
 //用途：屏幕初始化
   tft180_set_dir(TFT180_CROSSWISE);
   tft180_init();
   tft180_show_string(0, 0, "mt9v03x init.");
       while(1)
       {
           if(mt9v03x_init())
               tft180_show_string(0, 16, "mt9v03x reinit.");
           else
               break;
           system_delay_ms(1000);                                                  // 闪灯表示异常
       }
       tft180_show_string(0, 16, "init success.");
}

//-----------------------------------------------------------------------------------------------
//函数简介：显示图像
//-----------------------------------------------------------------------------------------------
void tft_show_image(void)
{
    tft180_displayimage03x((const uint8 *)mt9v03x_image, 160, 128);

}
//---------------------------------------------------------------------------------------------------
//函数简介：显示逆透视图像并绘制边界线
//          直接绘制原始跟踪点 left_edge/right_edge，可以看到完整边线轨迹
//---------------------------------------------------------------------------------------------------
void tft_show_warp_with_boundary(void)
{
    // 显示逆透视图像（WARP_IMAGE_W * WARP_IMAGE_H = 124 * 80）
    tft180_show_gray_image(0, 0, warp_image_ptr, WARP_IMAGE_W, WARP_IMAGE_H, WARP_IMAGE_W, WARP_IMAGE_H, 0);

    // 绘制左边线原始跟踪点（红色）
    for (int i = 0; i < left_edge_count; i++)
    {
        int x = left_edge[i][0];
        int y = left_edge[i][1];
        if (x >= 0 && x < WARP_IMAGE_W && y >= 0 && y < WARP_IMAGE_H)
        {
            tft180_draw_point(x, y, RGB565_RED);
        }
    }

    // 绘制右边线原始跟踪点（绿色）
    for (int i = 0; i < right_edge_count; i++)
    {
        int x = right_edge[i][0];
        int y = right_edge[i][1];
        if (x >= 0 && x < WARP_IMAGE_W && y >= 0 && y < WARP_IMAGE_H)
        {
            tft180_draw_point(x, y, RGB565_GREEN);
        }
    }

    // 绘制中线（蓝色）：从左右边线原始点构建逐行边界后取中点
    static uint8 tft_left[WARP_IMAGE_H];
    static uint8 tft_right[WARP_IMAGE_H];
    for (int y = 0; y < WARP_IMAGE_H; y++)
    {
        tft_left[y] = 255;
        tft_right[y] = 255;
    }
    for (int i = 0; i < left_edge_count; i++)
    {
        int y = left_edge[i][1];
        if (y >= 0 && y < WARP_IMAGE_H)
            tft_left[y] = (uint8)left_edge[i][0];
    }
    for (int i = 0; i < right_edge_count; i++)
    {
        int y = right_edge[i][1];
        if (y >= 0 && y < WARP_IMAGE_H)
            tft_right[y] = (uint8)right_edge[i][0];
    }
    for (int y = 0; y < WARP_IMAGE_H; y++)
    {
        if (tft_left[y] != 255 && tft_right[y] != 255)
        {
            int cx = (tft_left[y] + tft_right[y]) / 2;
            tft180_draw_point(cx, y, RGB565_BLUE);
        }
    }
}

/*
 * camera.c
 *
 *  Created on: 2026年1月15日
 *      Author: aaa
 */

#include "zf_common_headfile.h"

#pragma section all "cpu1_dsram"

uint8 xy_x1_boundary[BOUNDARY_NUM], xy_x2_boundary[BOUNDARY_NUM], xy_x3_boundary[BOUNDARY_NUM];
uint8 xy_y1_boundary[BOUNDARY_NUM], xy_y2_boundary[BOUNDARY_NUM], xy_y3_boundary[BOUNDARY_NUM];

uint8 x1_boundary[MT9V03X_H], x2_boundary[MT9V03X_H], x3_boundary[MT9V03X_H];
uint8 y1_boundary[MT9V03X_W], y2_boundary[MT9V03X_W], y3_boundary[MT9V03X_W];

// 图像备份数组，在发送前将图像备份再进行发送，这样可以避免图像出现撕裂的问题
IFX_ALIGN(4) uint8 image_copy[MT9V03X_H][MT9V03X_W];

#pragma section all restore

//-----------------------------------------------------------------------------------
//函数用途：发送图像到逐飞助手
//备注：模式介绍以及切换在"camera.h"中
//------------------------------------------------------------------------------------
void image_send_seekffree(void)
{


        // 设置逐飞助手使用DEBUG串口进行收发
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);

    #if(0 != INCLUDE_BOUNDARY_TYPE)
        int32 i=0;
    #endif

    #if(3 == INCLUDE_BOUNDARY_TYPE)
        int32 j=0;
    #endif

        gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);                             // 初始化 LED1 输出 默认高电平 推挽输出模式

        while(1)
           {
               if(mt9v03x_init())
                   gpio_toggle_level(LED1);                                            // 翻转 LED 引脚输出电平 控制 LED 亮灭 初始化出错这个灯会闪的很慢
               else
                   break;
               system_delay_ms(500);                                                  // 闪灯表示异常
           }



       #if(0 == INCLUDE_BOUNDARY_TYPE)
           // 发送总钻风图像信息(仅包含原始图像信息)
           seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image_copy[0], MT9V03X_W, MT9V03X_H);

       #elif(1 == INCLUDE_BOUNDARY_TYPE)
           // 发送总钻风图像信息(并且包含三条边界信息，边界信息只含有横轴坐标，纵轴坐标由图像高度得到，意味着每个边界在一行中只会有一个点)
           // 对边界数组写入数据
           for(i = 0; i < MT9V03X_H; i++)
           {
               x1_boundary[i] = 70 - (70 - 20) * (uint8)i / MT9V03X_H;
               x2_boundary[i] = MT9V03X_W / 2;
               x3_boundary[i] = 118 + (168 - 118) * (uint8)i / MT9V03X_H;
           }
           seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image_copy[0], MT9V03X_W, MT9V03X_H);
           seekfree_assistant_camera_boundary_config(X_BOUNDARY, MT9V03X_H, x1_boundary, x2_boundary, x3_boundary, NULL, NULL ,NULL);


       #elif(2 == INCLUDE_BOUNDARY_TYPE)
           // 发送总钻风图像信息(并且包含三条边界信息，边界信息只含有纵轴坐标，横轴坐标由图像宽度得到，意味着每个边界在一列中只会有一个点)
           // 通常很少有这样的使用需求
           // 对边界数组写入数据
           for(i = 0; i < MT9V03X_W; i++)
           {
               y1_boundary[i] = (uint8)i * MT9V03X_H / MT9V03X_W;
               y2_boundary[i] = MT9V03X_H / 2;
               y3_boundary[i] = (MT9V03X_W - (uint8)i) * MT9V03X_H / MT9V03X_W;
           }
           seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image_copy[0], MT9V03X_W, MT9V03X_H);
           seekfree_assistant_camera_boundary_config(Y_BOUNDARY, MT9V03X_W, NULL, NULL ,NULL, y1_boundary, y2_boundary, y3_boundary);


       #elif(3 == INCLUDE_BOUNDARY_TYPE)
           // 发送总钻风图像信息(并且包含三条边界信息，边界信息含有横纵轴坐标)
           // 这样的方式可以实现对于有回弯的边界显示
           j = 0;
           for(i = MT9V03X_H - 1; i >= MT9V03X_H / 2; i--)
           {
               // 直线部分
               xy_x1_boundary[j] = 34;
               xy_y1_boundary[j] = (uint8)i;

               xy_x2_boundary[j] = 47;
               xy_y2_boundary[j] = (uint8)i;

               xy_x3_boundary[j] = 60;
               xy_y3_boundary[j] = (uint8)i;
               j++;
           }

           for(i = MT9V03X_H / 2 - 1; i >= 0; i--)
           {
               // 直线连接弯道部分
               xy_x1_boundary[j] = 34 + (MT9V03X_H / 2 - (uint8)i) * (MT9V03X_W / 2 - 34) / (MT9V03X_H / 2);
               xy_y1_boundary[j] = (uint8)i;

               xy_x2_boundary[j] = 47 + (MT9V03X_H / 2 - (uint8)i) * (MT9V03X_W / 2 - 47) / (MT9V03X_H / 2);
               xy_y2_boundary[j] = 15 + (uint8)i * 3 / 4;

               xy_x3_boundary[j] = 60 + (MT9V03X_H / 2 - (uint8)i) * (MT9V03X_W / 2 - 60) / (MT9V03X_H / 2);
               xy_y3_boundary[j] = 30 + (uint8)i / 2;
               j++;
           }

           for(i = 0; i < MT9V03X_H / 2; i++)
           {
               // 回弯部分
               xy_x1_boundary[j] = MT9V03X_W / 2 + (uint8)i * (138 - MT9V03X_W / 2) / (MT9V03X_H / 2);
               xy_y1_boundary[j] = (uint8)i;

               xy_x2_boundary[j] = MT9V03X_W / 2 + i * (133 - MT9V03X_W / 2) / (MT9V03X_H / 2);
               xy_y2_boundary[j] = 15 + (uint8)i * 3 / 4;

               xy_x3_boundary[j] = MT9V03X_W / 2 + i * (128 - MT9V03X_W / 2) / (MT9V03X_H / 2);
               xy_y3_boundary[j] = 30 + (uint8)i / 2;
               j++;
           }
           seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image_copy[0], MT9V03X_W, MT9V03X_H);
           seekfree_assistant_camera_boundary_config(XY_BOUNDARY, BOUNDARY_NUM, xy_x1_boundary, xy_x2_boundary, xy_x3_boundary, xy_y1_boundary, xy_y2_boundary, xy_y3_boundary);


       #elif(4 == INCLUDE_BOUNDARY_TYPE)
           // 发送总钻风图像信息(并且包含三条边界信息，边界信息只含有横轴坐标，纵轴坐标由图像高度得到，意味着每个边界在一行中只会有一个点)
           // 对边界数组写入数据
           for(i = 0; i < MT9V03X_H; i++)
           {
               x1_boundary[i] = 70 - (70 - 20) * (uint8)i / MT9V03X_H;
               x2_boundary[i] = MT9V03X_W / 2;
               x3_boundary[i] = 118 + (168 - 118) * (uint8)i / MT9V03X_H;
           }
           seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, NULL, MT9V03X_W, MT9V03X_H);
           seekfree_assistant_camera_boundary_config(X_BOUNDARY, MT9V03X_H, x1_boundary, x2_boundary, x3_boundary, NULL, NULL ,NULL);


       #endif




                   mt9v03x_finish_flag = 0;

                   // 在发送前将图像备份再进行发送，这样可以避免图像出现撕裂的问题
                   memcpy(image_copy[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);

                   // 发送图像
                   seekfree_assistant_camera_send();



}



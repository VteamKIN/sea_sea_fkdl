/*
 * init.c
 * 外设与模块统一初始化入口
 * Author: aaa
 */
#include "zf_common_headfile.h"

volatile vuint8 camera_init_ok = 0;


void all_init(void)
{
    system_delay_init();            //延时初始化
    //wireless_uart_init();
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);       //串口初始化
    wireless_uart_init();
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIRELESS_UART);

    // 屏幕：必须先设横屏否则显示不全
    tft180_set_dir(TFT180_CROSSWISE_180);
    tft180_init();

    // 摄像头
    mt9v03x_init();

    // 编码器（内部会启动 CCU60_CH0 PIT 中断，周期 2 ms，驱动 encoder_read）
    encoder_init();

    // 电机 PWM
    motor_init();

    // 逆透视 LUT（一次性计算所有查表位置，运行时 WarpPerspective 只需查表）
    init_warp_lut();

    // IMU 相关
    imu660rb_init();
    my_imu_init();  // 陀螺仪初始化 + 零偏校准
    //imu_kalman_init_all(0.001f, 0.5f, 0.01f, 1.0f);

    // 按键 + 菜单
    //key_init(10);                                                         // 按键初始化（10ms 扫描周期）
    //pit_ms_init(CCU61_CH0, 10);                                           // 启动 10ms PIT 用于按键扫描
    //menu_init();                                                          // 菜单初始化（显示主菜单）

}





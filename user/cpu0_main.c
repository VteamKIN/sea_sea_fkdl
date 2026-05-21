#include "zf_common_headfile.h"

#pragma section all "cpu0_dsram"
// 将本语句与 #pragma section all restore 语句之间的全局变量都放在CPU0的RAM中

int core0_main(void)
{
    //系统级初始化
    clock_init();                   // 获取时钟频率（务必保留）
    debug_init();                   // 初始化默认调试串口

    all_init();

    cpu_wait_event_ready();         // 等待所有核心初始化完毕

    // 先启动负压风扇，等待负压建立后再允许轮子控制
    fan_set(3000);
    system_delay_ms(1000);           // 等待 1000ms 让负压建立
    control_enable_flag = 1;        // 允许 ISR 中的 control_process() 执行

    while (TRUE)
    {
        // 电机开环测试
        //motor_left(1500);
        //motor_right(-1500);
        //tft_show_image();
        //printf("yaw=%f\n", yaw_angle);
        //printf("%f, %f, %f\n",imu660ra_gyro_x, imu660ra_gyro_y, imu660ra_gyro_z);

        // 等待 CPU1 图像处理完成后再做同帧的后处理/显示
        if (cpu1_img_ready_flag)
        {

            //tft180_show_int(100, 85, img_process_time, 4);

            //tft180_show_int(0,0,left_slope_mutation,1);
            //tft180_show_int(20,0,right_slope_mutation,1);
            //tft180_show_int(50, 0, yaw_angle, 3);
            //printf("%d\n", error_image);
            // 示例见文件尾 DEBUG 区）
            //tft_show_warp_with_boundary();
            /*
            tft180_show_int(0, 20, dir_count, 2);
            tft180_show_int(0, 40, dir_count, 2);
            tft180_show_int(0, 60, dir_count, 2);
            tft180_show_int(30, 20, dir_count, 2);
            tft180_show_int(30, 40, dir_count, 2);
            tft180_show_int(30, 60, dir_count, 2);
            tft180_show_int(60, 20, dir_count, 2);
            tft180_show_int(60, 40, dir_count, 2);
            tft180_show_int(60, 60, dir_count, 2);
            */
            //tft_show_warp_with_boundary();
            __dsync();
            cpu1_img_ready_flag = 0;
        }
    }
}

#pragma section all restore

// ============================================================
// DEBUG 代码速查区
// 用途：调参/调试时从本区复制某一行到 core0_main 主循环相应位置启用。
// 保持 #if 0 关闭以避免编译进 release 固件。
// ============================================================
#if 0
{
    // 启动项占位（复制到 all_init 之后、主循环之前）
    system_delay_ms(1000);
    tft180_clear();

    // 主循环顶部（与 motor_left/motor_right 同级）
    param_tune_process();    // 上位机调参（非阻塞，文本协议 name=value\n）
    menu_process();          // 菜单处理（按键扫描已在 CCU61_CH0 PIT 中以 10ms 周期完成）
    wireless_uart_send_image(&mt9v03x_image[0][0], MT9V03X_IMAGE_SIZE);
    printf("raw=%d  dps=%f\r\n", imu660ra_gyro_y, imu660ra_gyro_transition(imu660ra_gyro_y));

    // if(cpu1_img_ready_flag) 内部：屏幕显示
    tft180_show_string(0, 85, "L:");  tft180_show_int(20, 85, error_left, 4);
    tft180_show_string(70, 85, "R:"); tft180_show_int(90, 85, error_right, 4);
    tft180_show_string(0, 100, "Img:"); tft180_show_int(30, 100, error_image, 4);
    tft180_show_int(100, 85, img_process_time, 4);
    tft180_show_int(100, 100, process_time, 5);

    tft180_show_string(0, 80, "D:");  tft180_show_int(16, 80, dir_count, 2);
    tft180_show_string(40, 80, "T:"); tft180_show_int(56, 80, current_target_dir, 3);
    tft180_show_string(88, 80, "A:"); tft180_show_int(104, 80, dir_advance_count, 3);
    tft180_show_string(0, 96, "J:");  tft180_show_int(16, 96, current_junction, 2);
    tft180_show_string(40, 96, "JD:"); tft180_show_int(64, 96, junction_detected, 1);
    tft180_show_string(88, 96, "RT:"); tft180_show_int(112, 96, road_type, 1);
    tft180_show_string(0, 112, "RJ:"); tft180_show_int(24, 112, raw_junction_debug, 2);
    tft180_show_string(64, 112, "E:"); tft180_show_int(80, 112, error_image, 4);

    // 上位机发送（Vofa / 逐飞助手）
    // dir_count 诊断套件：ch0~7 含义见各行参数
    Vofa_oscilloscope_send(dir_count, dir_advance_count, current_target_dir,
                           current_junction, junction_detected, road_type,
                           raw_junction_debug, (int)yaw_angle);
    Vofa_oscilloscope_send(error_image, road_type, current_junction, 0, 0, 0, 0, 0);
    Vofa_oscilloscope_send(img_process_time, 0, 0, 0, 0, 0, 0, 0);

    // 逐飞助手图传
    send_img_result_to_pc(SEND_MODE_IMAGE_ONLY);
    send_img_result_to_pc(SEND_MODE_IMAGE_WITH_BOUND);
    //image_send_seekffree();   // 阻塞式 + 含三种边界 demo，定义见 camera.c

    // printf 串口输出
    printf("process_time=%d img_process_time=%d\n", process_time, img_process_time);
    printf("encoder L=%d R=%d\n", encoder_data_l, encoder_data_r);
    printf("error_image=%d road_type=%d dir_count=%d choose=%d\n",
           error_image, road_type, dir_count, choose[dir_count]);
    printf("junction=%d detected=%d raw=%d\n",
           current_junction, junction_detected, raw_junction_debug);
    printf("left_edge_count=%d right_edge_count=%d\n", left_edge_count, right_edge_count);
}
#endif


#include "zf_common_headfile.h"
#pragma section all "cpu1_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU1的RAM中

// CPU1 图像处理结果就绪标志的实际定义（CPU0 通过 extern 引用）
volatile vuint8 cpu1_img_ready_flag = 0;

// 负压驱动就绪后才允许控制轮子（CPU0 置 1，ISR 中判断）
volatile vuint8 control_enable_flag = 0;


// 工程导入到软件之后，应该选中工程然后点击refresh刷新一下之后再编译
// 工程默认设置为关闭优化，可以自己右击工程选择properties->C/C++ Build->Setting
// 然后在右侧的窗口中找到C/C++ Compiler->Optimization->Optimization level处设置优化等级
// 一般默认新建立的工程都会默认开2级优化，因此大家也可以设置为2级优化

// 对于TC系列默认是不支持中断嵌套的，希望支持中断嵌套需要在中断内使用 enableInterrupts(); 来开启中断嵌套
// 简单点说实际上进入中断后TC系列的硬件自动调用了 disableInterrupts(); 来拒绝响应任何的中断，因此需要我们自己手动调用 enableInterrupts(); 来开启中断的响应。


// **************************** 代码区域 ****************************
void core1_main(void)
{
    disable_Watchdog();                     // 关闭看门狗
    interrupt_global_enable(0);             // 打开全局中断

    // 此处编写用户代码 例如外设初始化代码等

    cpu_wait_event_ready();                 // 等待所有核心初始化完毕

    car_running = 0;                        // wait for KEY3 launch

    // 启动 4ms PIT 定时器，图像处理 + 控制闭环在 CCU61_CH1 ISR 中完成

    pit_ms_init(CCU61_CH1, 4);

    while (TRUE)
    {

    }
}
#pragma section all restore

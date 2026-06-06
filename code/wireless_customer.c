/*
 * wireless_customer.c
 *
 *  Created on: 2024年12月18日
 *      Author: zqq
 * 备注：这个库是用于使用无线串口时在逐飞助手上显示波形的，大部分代码都是从逐飞的demo中移植
 * 有vofa的函数，优先使用vofa吧，能传超大的数和浮点数，还能实时画轨迹，比逐飞助手好用多了
 */
#include "zf_common_typedef.h"
#include "zf_common_headfile.h"

#pragma section all "cpu0_dsram"
uint8 vofa_tail[4] = {0x00, 0x00, 0x80, 0x7f};//vofa中justfloat协议的帧尾
#pragma section all restore
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     vofa上位机相关功能的初始化
// 参数说明     void            无
// 返回参数     void            无
// 使用示例     vofa_init();
// 函数备注     需要使用上位机观察波形时使用
//-------------------------------------------------------------------------------------------------------------------
void vofa_init(void)
{
    wireless_uart_init();
    /* 请注意无线串口引脚的配置
     * 此处只使用了基础的TX,RX引脚，并没有使用RTS流控引脚
     * 请自行手动修改更改逐飞库中的代码，删除RTS相关的代码，否则无法发送数据（具体怎么改可以参考本工程的zf_device_wireless_uart.c）
     * 由于没有使用RTS，请关闭自动波特率
     */
    /*
     * vofa设置
     * 数据引擎选择justfloat
     */
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     向上位机发送数据函数
// 参数说明     data            要发送的数据，不使用就设为0
// 返回参数     void            无
// 使用示例     vofa_send();
// 函数备注
//-------------------------------------------------------------------------------------------------------------------
//vofa_data_struct vofa_data;



void Vofa_oscilloscope_send(float Data1,float Data2,float Data3,float Data4,float Data5,float Data6,float Data7,float Data8)
{
    static float vofa_data[8]={0};
    vofa_data[7]=Data8;
    vofa_data[6]=Data7;
    vofa_data[5]=Data6;
    vofa_data[4]=Data5;
    vofa_data[3]=Data4;
    vofa_data[2]=Data3;
    vofa_data[1]=Data2;
    vofa_data[0]=Data1;

    debug_send_buffer(vofa_data,8*sizeof(float));
//
    debug_send_buffer(vofa_tail, 4*sizeof(uint8));//发送vofa中justfloat协议的帧尾

    //wireless_uart_send_buffer(vofa_data,8*sizeof(float));
    //wireless_uart_send_buffer(vofa_tail, 4);

}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     从 Vofa 上位机接收 8 通道 justfloat 协议数据（非阻塞 + 滑窗解析）
// 参数说明     DATA1..DATA8    输出指针，解析成功时写入最新一帧 8 个 float
// 返回参数     void            无（未收到完整帧时，*DATAx 保持原值不变）
// 使用示例     Vofa_oscilloscope_receive(&v1,&v2,&v3,&v4,&v5,&v6,&v7,&v8);
// 函数备注     justfloat 协议：8 × float (32B 小端 IEEE754) + 4B 帧尾 {0x00,0x00,0x80,0x7f}
//             调用者应在主循环或定时任务中周期性调用，本函数立即返回不会阻塞
//-------------------------------------------------------------------------------------------------------------------
#pragma section all "cpu0_dsram"
#define VOFA_FRAME_BYTES   36                       // 8*4 + 4 帧尾
static uint8  parser_buffer[VOFA_FRAME_BYTES] = {0};// 滑窗缓冲：始终持有最新 36 字节
static uint8  parser_index = 0;                     // 当前已填充字节数 (0~36)
static Vofa_receive Receive_data[Vofa_Chanel];      // 解析结果（按通道存）
static uint8  parser_state = 0;                     // 0=等待填满, 1=滑窗校验
#pragma section all restore

void Vofa_oscilloscope_receive(float *DATA1,float *DATA2,float *DATA3,float *DATA4,
                               float *DATA5,float *DATA6,float *DATA7,float *DATA8)
{
    uint8 rx_buf[VOFA_FRAME_BYTES];
    // 一次最多读 36 字节，避免 FIFO 一次溢出过多
    uint32 got = wireless_uart_read_buffer(rx_buf, VOFA_FRAME_BYTES);
    uint32 i;

    for (i = 0; i < got; i++)
    {
        if (parser_index < VOFA_FRAME_BYTES)
        {
            parser_buffer[parser_index++] = rx_buf[i];
        }
        else
        {
            // 滑窗：丢最早一字节，左移一位再追加
            uint32 k;
            for (k = 0; k < VOFA_FRAME_BYTES - 1; k++)
                parser_buffer[k] = parser_buffer[k + 1];
            parser_buffer[VOFA_FRAME_BYTES - 1] = rx_buf[i];
        }

        // 缓冲满后开始尝试匹配帧尾
        if (parser_index >= VOFA_FRAME_BYTES)
        {
            parser_state = 1;
            if (parser_buffer[32] == 0x00 &&
                parser_buffer[33] == 0x00 &&
                parser_buffer[34] == 0x80 &&
                parser_buffer[35] == 0x7f)
            {
                // 匹配成功：拆出 8 个 float
                uint8 ch, b;
                for (ch = 0; ch < Vofa_Chanel; ch++)
                {
                    for (b = 0; b < 4; b++)
                        Receive_data[ch].byte[b] = parser_buffer[ch * 4 + b];
                }
                if (DATA1) *DATA1 = Receive_data[0].f;
                if (DATA2) *DATA2 = Receive_data[1].f;
                if (DATA3) *DATA3 = Receive_data[2].f;
                if (DATA4) *DATA4 = Receive_data[3].f;
                if (DATA5) *DATA5 = Receive_data[4].f;
                if (DATA6) *DATA6 = Receive_data[5].f;
                if (DATA7) *DATA7 = Receive_data[6].f;
                if (DATA8) *DATA8 = Receive_data[7].f;

                // 帧已消费，重置缓冲，准备接下一帧
                parser_index = 0;
                parser_state = 0;
            }
        }
    }
}

/*
 * wireless_customer.h
 *
 *  Created on: 2024年12月18日
 *      Author: zqq
 * 备注：这个库是用于使用无线串口时在逐飞助手上显示波形的，大部分代码都是从逐飞的demo中移植
 * 有vofa的函数，优先使用vofa吧，能传超大的数和浮点数，还能实时画轨迹，比逐飞助手好用多了
 *
 *
 * 新版开源库自带逐飞助手文件，于是删除该文件中逐飞助手部分
 */

#ifndef CODE_WIRELESS_CUSTOMER_H_
#define CODE_WIRELESS_CUSTOMER_H_

#include "zf_common_typedef.h"
#include "zf_common_headfile.h"

#define Vofa_Chanel 8
typedef union{
        float f;
        uint8 byte[4];
}Vofa_receive;

extern uint16 tly_time;
void vofa_init(void);
void Vofa_oscilloscope_send(float Data1,float Data2,float Data3,float Data4,float Data5,float Data6,float Data7,float Data8);
void Vofa_oscilloscope_receive(float *DATA1,float *DATA2,float *DATA3,float *DATA4,float *DATA5,float *DATA6,float *DATA7,float *DATA8);

#endif /* CODE_WIRELESS_CUSTOMER_H_ */

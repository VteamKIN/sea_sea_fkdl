/*
 * menu.h
 *
 *  Created on: 2026年3月14日
 *      Author: aaa
 */

#ifndef CODE_MENU_H_
#define CODE_MENU_H_

#include "zf_common_headfile.h"

/*******************************************************************************
 * 菜单参数调节接口
 *******************************************************************************/

/**
 * 菜单初始化
 * 初始化TFT屏幕和菜单变量
 */
void menu_init(void);

/**
 * 菜单处理函数
 * 处理按键输入和参数调节
 * 需要在主循环中调用
 */
void menu_process(void);

/**
 * 显示当前PID参数
 */
void menu_show_pid(void);

#endif /* CODE_MENU_H_ */

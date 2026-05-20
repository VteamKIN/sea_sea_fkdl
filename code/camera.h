/*
 * camera.h
 *
 *  Created on: 2026年1月15日
 *      Author: aaa
 */

#ifndef CODE_CAMERA_H_
#define CODE_CAMERA_H_

//0 不包含边界
//1 包含左边界和右边界，但左边界和右边界均不能与图像边界重合，即左边界>0，右边界<图像宽度-1
//2 包含左边界和右边界，左边界可以等于0，右边界可以等于图像宽度-1，但是左边界和右边界之间不能为0
//3 包含左边界和右边界，左边界可以等于0，右边界可以等于图像宽度-1，左边界和右边界之间可以为0，但是左边界和右边界之间不能为0
//4 包含左边界和右边界，左边界可以等于0，右边界可以等于图像宽度-1，左边界和右边界之间可以为0
#define INCLUDE_BOUNDARY_TYPE   0

// 定义边界数量常量
#define BOUNDARY_NUM            (MT9V03X_H * 3 / 2)

#define LED1                    (P21_5 )

void image_send_seekffree(void);
#endif /* CODE_CAMERA_H_ */

#ifndef __AGV_H
#define __AGV_H
#include "sys.h"

void AGV_Data_Open(void);	// AGV开启数据传回
void AGV_Data_Stop(void);	// AGV停止数据回传
void AGV_GO(uint8_t speed, uint16_t mp);	// AGV前进 参数：速度 码盘
void AGV_Break(uint8_t speed, uint16_t mp);	// AGV后退 参数：速度 码盘
void AGV_Track(uint8_t speed);	// AGV循迹 参数：速度
void AGV_Reght(uint8_t speed);	// AGV右转 参数：速度
void AGV_Left(uint8_t speed);		// AGV左转 参数：速度
void AGV_STOP(void);		// AGV停止
void AGV_Beep(uint8_t mode);		// AGV_Zigbee开启蜂鸣器 参数：开关
void AGV_Infrared(uint8_t *Src);	// AGV_Zigbee发送红外
void AGV_SendData(uint8_t Pri,uint8_t Sec1,uint8_t Sec2,uint8_t Sec3);	// AGV发送数据
void AGV_Wheel_L45(uint8_t speed,uint16_t time);	// AGV左转45 参数：速度
void AGV_Wheel_R45(uint8_t speed,uint16_t time);	// AGV右转45 参数：速度
void AGV_Thread(uint8_t mode);			// AGV自动驾驶
void AGV_GetThread(uint8_t mode);		// AGV全自动完成标志获取
	
#endif


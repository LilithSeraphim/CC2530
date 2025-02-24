#ifndef __DRIVE_H_
#define __DRIVE_H_
#include "sys.h"

void Gate_Open_Zigbee(void);			// 道闸闸门开启
void Gate_Show_Zigbee(char *Licence);	// 道闸系统标志物显示车牌
void LED_Date_Zigbee(uint8_t One, uint8_t Two, uint8_t Three, uint8_t rank);	// LED显示标志物显示数据
void LED_Dis_Zigbee(uint16_t dis);		// LED显示标志物显示测距信息
void Rotate_show_Inf(char* src, char x, char y);	// 立体显示标志物显示车牌数据
void Rotate_Dis_Inf(uint16_t dis);      // 立体显示标志物显示距离信息
void YY_Play_Zigbee(char *p);			// 语音播报标志物播报指定文本信息
void YY_Comm_Zigbee(uint8_t Primary, uint8_t Secondary); // 控制语音播报标志物播报语音控制命令
void TFT_Test_Zigbee(char Device,uint8_t Pri,uint8_t Sec1,uint8_t Sec2,uint8_t Sec3);	// TFT显示标志物控制指令
void TFT_Dis_Zigbee(char Device,uint16_t dis);		// 智能TFT显示器显示距离信息
void TFT_Show_Zigbee(char Device,char *Licence);	// TFT显示器显示车牌
void Garage_Test_Zigbee(char Device,uint8_t Pri,uint8_t Sec1);		// 立体车库标志物控制函数
void Garage_Cont_Zigbee(char Device,uint8_t Floor);		// 立体车库到达指定车库
void RegtLight_Inf(uint8_t gear);		// 自动调节光照强度函数
uint8_t Light_Inf(uint8_t gear);		// 自动调节光照强度函数
void ETC_Get_Zigbee(void);				// ETC系统检测
void Test_Zigbee(uint8_t Head,uint8_t Pri,uint8_t Sec1,uint8_t Sec2,uint8_t Sec3);
void Send_Android(uint8_t Pri,uint8_t Sec);

void Car_Go(uint8_t speed, uint16_t temp);   // 主车前进 参数：速度/码盘
void Car_Back(uint8_t speed, uint16_t temp); // 主车后退 参数：速度/码盘
void Car_L45(int8_t speed, uint16_t times);		// 左旋转 参数：旋转时间
void Car_Left(uint8_t speed);       // 主车左转 参数：速度
void Car_R45(int8_t speed, uint16_t tims);		// 右旋转 参数：旋转时间
void Car_Right(uint8_t speed);       // 主车右转 参数：速度
void Car_Time_Track(uint8_t speed, uint16_t tims);		// 小车循迹 参数：速度，时间
void Car_Track(uint8_t speed);   // 主车循迹 参数：速度
void CarThread_Go(uint8_t speed, uint16_t temp);   // 主车前进 参数：速度/码盘
void CarThread_Track(uint8_t speed);   // 主车循迹 参数：速度

#endif


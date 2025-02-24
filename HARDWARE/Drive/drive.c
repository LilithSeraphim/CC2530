#include "drive.h"
#include <stdio.h>
#include "stm32f4xx.h"
#include "delay.h"
#include "infrared.h"
#include "cba.h"
#include "ultrasonic.h"
#include "canp_hostcom.h"
#include "hard_can.h"
#include "bh1750.h"
#include "syn7318.h"
#include "power_check.h"
#include "can_user.h"
#include "data_base.h"
#include "roadway_check.h"
#include "tba.h"
#include "data_base.h"
#include "swopt_drv.h"
#include "uart_a72.h"
#include "Can_check.h"
#include "delay.h"
#include "can_user.h"
#include "Timer.h"
#include "Rc522.h"
#include "string.h"

/*************************** 自定义变量 *******************************/
static uint8_t Zigbee[8];           // Zigbee发送数据缓存
static uint8_t Infrared[6];         // 红外发送数据缓存
static uint8_t YY_Init[5] = {0xFD, 0x00, 0x00, 0x01, 0x01};
uint16_t tim_a,tim_b;

/**********************************************************************/

/**********************************************************************
 * 函 数 名 ：  道闸系统闸门开启控制
 * 参    数 ：  无
 * 返 回 值 ：  无
 * 简    例 ：  Gate_Open_Zigbee();     // 道闸闸门 -> 开启
**********************************************************************/
void Gate_Open_Zigbee(void)
{
    Send_ZigbeeData_To_Fifo(Gate_Open, 8);		// 道闸 -> 开启
	delay_ms(300);
	Send_ZigbeeData_To_Fifo(Gate_GetStatus, 8);	// 道闸 状态查询
	delay_ms(100);
#if 0
	tim_a = 0;
	tim_b = 0;
    Stop_Flag = 0;
    while(Stop_Flag != 0x05)
    {
        delay_ms(1);
        tim_a++;
        if(tim_a >= 500)
        {
            tim_a = 0;
			tim_b++;
			if (tim_b >= 5) { break;}
			Send_ZigbeeData_To_Fifo(Gate_Open, 8);		// 道闸 -> 开启
			delay_ms(300);
			Send_ZigbeeData_To_Fifo(Gate_GetStatus, 8);	// 道闸 状态查询
			delay_ms(100);
        }
        if(Zigbee_Rx_flag)      // 判读Zigbee数据回传
        {
			Zigbee_Rx_flag = 0;
            if(Zigb_Rx_Buf[1] == 0x03)      // 道闸
            {
                if(Zigb_Rx_Buf[2] == 0x01)
                {
                    Stop_Flag = Zigb_Rx_Buf[4];
                }
            }
        }
    }
#endif
}

/**********************************************************************
 * 函 数 名 ：  道闸系统标志物显示车牌
 * 参    数 ：  *Licence -> 车牌数据（ASICC）
 * 返 回 值 ：  无
 * 简    例 ：  Gate_Show_Zigbee("A123B4");
**********************************************************************/
void Gate_Show_Zigbee(char *Licence)
{
    Zigbee[0] = 0x55;
    Zigbee[1] = 0x03;
    Zigbee[2] = 0x10;
    Zigbee[3] = *(Licence + 0);     // 车牌数据【1】
    Zigbee[4] = *(Licence + 1);     // 车牌数据【2】
    Zigbee[5] = *(Licence + 2);     // 车牌数据【3】
    Zigbee[6] = (Zigbee[2] + Zigbee[3] + Zigbee[4] + Zigbee[5]) % 256;
    Zigbee[7] = 0xBB;
    Send_ZigbeeData_To_Fifo(Zigbee, 8);     // 发送Zigbee数据
    delay_ms(400);
    Zigbee[2] = 0x11;
    Zigbee[3] = *(Licence + 3);     // 车牌数据【4】
    Zigbee[4] = *(Licence + 4);     // 车牌数据【5】
    Zigbee[5] = *(Licence + 5);     // 车牌数据【6】
    Zigbee[6] = (Zigbee[2] + Zigbee[3] + Zigbee[4] + Zigbee[5]) % 256;
    Zigbee[7] = 0xBB;
    Send_ZigbeeData_To_Fifo(Zigbee, 8);     // 发送Zigbee数据
    delay_ms(100);
}

/**********************************************************************
 * 函 数 名 ：  LED显示标志物显示数据
 * 参    数 ：  One,Two,Three   数据（十六进制显示格式）
                rank    1 -> 第一行数码管显示数据
                        2 -> 第二行数码管显示数据（默认）
 * 返 回 值 ：  无
 * 简    例 ：  LED_Date_Zigbee(0x12,0x34,0x56,0x01);
**********************************************************************/
void LED_Date_Zigbee(uint8_t One, uint8_t Two, uint8_t Three, uint8_t rank)
{
    Zigbee[0] = 0x55;
    Zigbee[1] = 0x04;
    if(rank == 1)  Zigbee[2] = 0x01;
    else  Zigbee[2] = 0x02;
    Zigbee[3] = One;
    Zigbee[4] = Two;
    Zigbee[5] = Three;
    Zigbee[6] = (Zigbee[2] + Zigbee[3] + Zigbee[4] + Zigbee[5]) % 256;
    Zigbee[7] = 0xBB;
    Send_ZigbeeData_To_Fifo(Zigbee, 8);
    delay_ms(100);
}

/**********************************************************************
 * 函 数 名 ：  LED显示标志物显示测距信息
 * 参    数 ：  dis 距离值
 * 返 回 值 ：  无
 * 简    例 ：  LED_Dis_Zigbee(123);
**********************************************************************/
void LED_Dis_Zigbee(uint16_t dis)
{
    Zigbee[0] = 0x55;
    Zigbee[1] = 0x04;
    Zigbee[2] = 0x04;
    Zigbee[3] = 0x00;
    Zigbee[4] = dis / 100 % 10;
    Zigbee[5] = ((dis / 10 % 10) * 16 + dis % 10);
    Zigbee[6] = (Zigbee[2] + Zigbee[3] + Zigbee[4] + Zigbee[5]) % 256;
    Zigbee[7] = 0xBB;
    Send_ZigbeeData_To_Fifo(Zigbee, 8);  //发送Zigbee数据
    delay_ms(100);
}

/**********************************************************************
 * 函 数 名 ：  立体显示标志物显示车牌数据
 * 参    数 ：  *src    车牌数据（ASICC）
                x,y     坐标信息
 * 返 回 值 ：  无
 * 简    例 ：  Rotate_show_Inf("A123B4",'C','5');
**********************************************************************/
void Rotate_show_Inf(char* src, char x, char y)
{
    Infrared[0] = 0xFF;			// 起始位
    Infrared[1] = 0x20;			// 模式
    Infrared[2] = *(src + 0);	// 数据【1】
    Infrared[3] = *(src + 1);	// 数据【2】
    Infrared[4] = *(src + 2);	// 数据【3】
    Infrared[5] = *(src + 3);	// 数据【4】
    Infrared_Send(Infrared, 6);
    delay_ms(500);
    Infrared[1] = 0x10;			// 模式
    Infrared[2] = *(src + 4);	// 数据【1】
    Infrared[3] = *(src + 5);	// 数据【2】
    Infrared[4] = x;			// 数据【3】
    Infrared[5] = y;			// 数据【4】
    Infrared_Send(Infrared, 6);
    delay_ms(10);
}

/**********************************************************************
 * 函 数 名 ：  立体显示标志物显示距离信息（单位：ms）
 * 参    数 ：  dis  测距信息（四舍五入）
 * 返 回 值 ：  无
 * 简    例 ：  Rotate_Dis_Inf(123);
**********************************************************************/
void Rotate_Dis_Inf(uint16_t dis)
{
    uint16_t csb = dis; //缓存超声波数据值

    csb += 5;   //四舍五入
    Infrared[0] = 0xFF;
    Infrared[1] = 0x11; //显示距离模式
    Infrared[2] = 0x30 + (uint8_t)(csb / 100 % 10); //距离十位--cm
    Infrared[3] = 0x30 + (uint8_t)(csb / 10 % 10); //距离个位--cm
    Infrared[4] = 0x00;
    Infrared[5] = 0x00;
    Infrared_Send(Infrared, 6);
    delay_ms(100);
}

/**********************************************************************
 * 函 数 名 ：  控制语音播报标志物播报指定文本信息
 * 参    数 ：  *p  --> 需要发送的数据
 * 返 回 值 ：  无
 * 简    例 ：  YY_Play_Zigbee("北京欢迎您");
**********************************************************************/
void YY_Play_Zigbee(char *p)
{
    uint16_t p_len = strlen(p);             // 文本长度

    YY_Init[1] = 0xff & ((p_len + 2) >> 8); // 数据区长度高八位
    YY_Init[2] = 0xff & (p_len + 2);        // 数据区长度低八位
    Send_ZigbeeData_To_Fifo(YY_Init, 5);
    Send_ZigbeeData_To_Fifo((uint8_t *)p, p_len);
    delay_ms(100);
}

/**********************************************************************
 * 函 数 名 ：  控制语音播报标志物播报语音控制命令
 * 参    数 ：  Primary   -> 主指令
                Secondary -> 副职令
                详见附录1
 * 返 回 值 ：  无
 * 简    例 ：  YY_Comm_Zigbee(0x20, 0x01);     // 语音播报随机语音命令

附录1：
-----------------------------------------------------------------------
| Primary | Secondary | 说明
|---------|-----------|------------------------------------------------
|  0x10   |  0x02     | 向右转弯
|         |  0x03     | 禁止右转
|         |  0x04     | 左侧行驶
|         |  0x05     | 左行被禁
|         |  0x06     | 原地掉头
|---------|-----------|------------------------------------------------
|  0x20   |  0x01     | 随机指令
|---------|-----------|------------------------------------------------
***********************************************************************/
void YY_Comm_Zigbee(uint8_t Primary, uint8_t Secondary)
{
    Zigbee[0] = 0x55;
    Zigbee[1] = 0x06;
    Zigbee[2] = Primary;
    Zigbee[3] = Secondary;
    Zigbee[4] = 0x00;
    Zigbee[5] = 0x00;
    Zigbee[6] = (Zigbee[2] + Zigbee[3] + Zigbee[4] + Zigbee[5]) % 256;
    Zigbee[7] = 0xBB;
    Send_ZigbeeData_To_Fifo(Zigbee, 8);
}

/********************************************************************************
 * 函 数 名 ：  TFT显示标志物控制指令
 * 参    数 ：  Device -> 选择设备
				Pri	   -> 主指令
                Sec1   -> 副职令【1】
				Sec2   -> 副职令【2】
				Sec3   -> 副职令【3】
                详见附录1
 * 返 回 值 ：  无
 * 简    例 ：  TFT_Test_Zigbee('A',0x40,0xA1,0xB2,0xC3);    // TFT显示器显示图形信息

附录1：
--------------------------------------------------------------------------------
| 主指令 | 副指令[1] | 副指令[2] | 副指令[3] |			说明
|--------|-----------|-----------|-----------|-----------------------------------
|  0x10  |	 0x00	 | 0x01~0x20 |   0x00    | 由第二副指令指定显示那张图片
|        |	 0x01    |	 0x00	 |   0x00    | 图片向上翻页
|        |   0x02    |	 0x00	 |   0x00    | 图片向下翻页
|		 |   0x03    |	 0x00    |   0x00    | 图片自动向下翻页显示，间隔时间 10S
|--------|-----------|-----------|-----------|-----------------------------------
|  0x20  |	 0xXX    |	 0xXX	 |   0xXX	 | 车牌前三位数据（ASCII）
|--------|-----------|-----------|-----------|-----------------------------------
|  0x21  |	 0xXX	 |	 0xXX	 |   0xXX	 | 车牌后三位数据（ASCII）
|--------|-----------|-----------|-----------|-----------------------------------
|  0x30  |	 0x00	 |	 0x00	 |   0x00	 | 计时模式关闭
|	 	 |	 0x01	 |	 0x00 	 |   0x00	 | 计时模式打开
| 		 |	 0x02	 |	 0x00	 |   0x00	 | 计时模式清零
|--------|-----------|-----------|-----------|-----------------------------------
|  0x40  |	 0xXX	 |	 0xXX    |   0xXX	 | 六位显示数据（HEX格式）
|--------|-----------|-----------|-----------|-----------------------------------
|  0x50  |	 0x00	 |	 0x0X	 |   0xXX	 | 距离显示模式（十进制）
--------------------------------------------------------------------------------
********************************************************************************/
void TFT_Test_Zigbee(char Device,uint8_t Pri,uint8_t Sec1,uint8_t Sec2,uint8_t Sec3)
{
	Zigbee[0] = 0x55;
	if (Device == 'B') { Zigbee[1] = 0x08; }
	else { Zigbee[1] = 0x0B; }
	Zigbee[2] = Pri;
	Zigbee[3] = Sec1;
	Zigbee[4] = Sec2;
	Zigbee[5] = Sec3;
	Zigbee[6] = (Zigbee[2]+Zigbee[3]+Zigbee[4]+Zigbee[5])%256;
	Zigbee[7] = 0xBB;
	Send_ZigbeeData_To_Fifo(Zigbee, 8);
	delay_ms(100);
}

/**********************************************************************
 * 函 数 名 ：  智能TFT显示器显示距离信息
 * 参    数 ：  dis	距离值
 * 返 回 值 ：  无
 * 简    例 ：  TFT_Dis_Zigbee('A',123);
**********************************************************************/
void TFT_Dis_Zigbee(char Device,uint16_t dis)
{
	Zigbee[0] = 0x55;
	if (Device == 'B') { Zigbee[1] = 0x08; }
	else { Zigbee[1] = 0x0B; }
	Zigbee[2] = 0x50;
	Zigbee[3] = 0x00;
	Zigbee[4] = dis/100%10;
	Zigbee[5] = ((dis/10%10)*16+dis%10);
	Zigbee[6] = (Zigbee[2]+Zigbee[3]+Zigbee[4]+Zigbee[5])%256;
	Zigbee[7] = 0xBB;
	Send_ZigbeeData_To_Fifo(Zigbee, 8);  //发送Zigbee数据
	delay_ms(100);
}


/**********************************************************************
 * 函 数 名 ：  TFT显示器显示车牌
 * 参    数 ：  *Licence -> 车牌数据（ASICC）
 * 返 回 值 ：  无
 * 简    例 ：  TFT_Show_Zigbee('A',"A123B4");
**********************************************************************/
void TFT_Show_Zigbee(char Device,char *Licence)
{
    Zigbee[0] = 0x55;
    if (Device == 'B') { Zigbee[1] = 0x08; }
	else { Zigbee[1] = 0x0B; }
    Zigbee[2] = 0x20;
    Zigbee[3] = *(Licence + 0);     // 车牌数据【1】
    Zigbee[4] = *(Licence + 1);     // 车牌数据【2】
    Zigbee[5] = *(Licence + 2);     // 车牌数据【3】
    Zigbee[6] = (Zigbee[2] + Zigbee[3] + Zigbee[4] + Zigbee[5]) % 256;
    Zigbee[7] = 0xBB;
    Send_ZigbeeData_To_Fifo(Zigbee, 8);     // 发送Zigbee数据
    delay_ms(500);
    Zigbee[2] = 0x21;
    Zigbee[3] = *(Licence + 3);     // 车牌数据【4】
    Zigbee[4] = *(Licence + 4);     // 车牌数据【5】
    Zigbee[5] = *(Licence + 5);     // 车牌数据【6】
    Zigbee[6] = (Zigbee[2] + Zigbee[3] + Zigbee[4] + Zigbee[5]) % 256;
    Zigbee[7] = 0xBB;
    Send_ZigbeeData_To_Fifo(Zigbee, 8);     // 发送Zigbee数据
    delay_ms(100);
}

/********************************************************************************
 * 函 数 名 ：  立体车库标志物控制函数
 * 参    数 ：  Device -> 选择设备
				Pri	   -> 主指令
                Sec1   -> 副职令【1】
                详见附录1
 * 返 回 值 ：  无
 * 简    例 ：  Garage_Test_Zigbee('A',0x01,0x01);    // 立体车库A 到达第一层

附录1：
--------------------------------------------------------------------------------
| 主指令 | 副指令[1] | 副指令[2] | 副指令[3] |			说明
|--------|-----------|-----------|-----------|----------------------------------
|  0x01  |   0x01    |   0x00    |   0x00    | 到达第一层
|        |   0x02    |   0x00    |   0x00    | 到达第二层
|        |   0x03    |   0x00    |   0x00    | 到达第三层
|        |   0x04    |   0x00    |   0x00    | 到达第四层
|--------|-----------|-----------|-----------|----------------------------------
|  0x02  |   0x01    |   0x00    |   0x00    | 请求返回车库位于第几层
|        |   0x02    |   0x00    |   0x00    | 请求返回前后侧红外状态
|--------|-----------|-----------|-----------|----------------------------------
|  0x03  |   0x01    |   0x01    |   0x00    | 返回车库位于第一层
|        |           |   0x02    |   0x00    | 返回车库位于第二层
|        |           |   0x03    |   0x00    | 返回车库位于第三层
|        |           |   0x04    |   0x00    | 返回车库位于第四层
|        |-----------|-----------|-----------|----------------------------------
|        |   0x02    |0x01 触发  |0x01 触发  | 返回车库前后侧红外对管状态
|        |           |0x02 未触发|0x02 未触发| 
|        |           | - 前侧 -  | - 后侧 -  | 
--------------------------------------------------------------------------------
********************************************************************************/
void Garage_Test_Zigbee(char Device,uint8_t Pri,uint8_t Sec1)
{
	Zigbee[0] = 0x55;
	if (Device == 'B') { Zigbee[1] = 0x08; }
	else { Zigbee[1] = 0x0D; }
	Zigbee[2] = Pri;
	Zigbee[3] = Sec1;
	Zigbee[4] = 0x00;
	Zigbee[5] = 0x00;
	Zigbee[6] = (Zigbee[2]+Zigbee[3]+Zigbee[4]+Zigbee[5])%256;
	Zigbee[7] = 0xBB;
	Send_ZigbeeData_To_Fifo(Zigbee, 8);
	delay_ms(100);
}

/**********************************************************************
 * 函 数 名 ：  立体车库到达指定车库
 * 参    数 ：  Device -> 选择设备
				Floor  -> 层数
 * 返 回 值 ：  无
 * 简    例 ：  Garage_Cont_Zigbee('A', 1);	// 立体车库A 到达第一层
**********************************************************************/
void Garage_Cont_Zigbee(char Device,uint8_t Floor)
{
	tim_a = 150;
	tim_b = 0;
	while(1)
	{
		tim_a++;
		delay_ms(10);
		if(!(tim_a%80))
		{
			if (Device == 'B')
			{
				Send_ZigbeeData_To_Fifo(GarageB_Get_Floor, 8);	// 请求车库B返回位于第几层
			} else 
			{
				Send_ZigbeeData_To_Fifo(GarageA_Get_Floor, 8);	// 请求车库A返回位于第几层
			}
		}
		if(tim_a > 200)
		{
			tim_a = 0;
			tim_b++;
//			if(tim_b >= 15) { break; }
			
			if ((Device == 'A') && (Floor == 1))
			{
				Send_ZigbeeData_To_Fifo(GarageA_To1, 8);	// 立体车库A 到达第1层
			} else if ((Device == 'A') && (Floor == 2))
			{
				Send_ZigbeeData_To_Fifo(GarageA_To2, 8);	// 立体车库A 到达第2层
			} else if ((Device == 'A') && (Floor == 3))
			{
				Send_ZigbeeData_To_Fifo(GarageA_To3, 8);	// 立体车库A 到达第3层
			} else if ((Device == 'A') && (Floor == 4))
			{
				Send_ZigbeeData_To_Fifo(GarageA_To4, 8);	// 立体车库A 到达第4层
			} else if ((Device == 'B') && (Floor == 1))
			{
				Send_ZigbeeData_To_Fifo(GarageB_To1, 8);	// 立体车库B 到达第1层
			} else if ((Device == 'B') && (Floor == 2))
			{
				Send_ZigbeeData_To_Fifo(GarageB_To2, 8);	// 立体车库B 到达第2层
			} else if ((Device == 'B') && (Floor == 3))
			{
				Send_ZigbeeData_To_Fifo(GarageB_To3, 8);	// 立体车库B 到达第3层
			} else if ((Device == 'B') && (Floor == 4))
			{
				Send_ZigbeeData_To_Fifo(GarageA_To4, 8);	// 立体车库B 到达第4层
			} else
			{
				Send_ZigbeeData_To_Fifo(GarageA_To1, 8);	// 立体车库A 到达第1层
			}
		}
		if(Zigbee_Rx_flag == 1)	 //zigbee返回信息
		{
			Zigbee_Rx_flag =0;
			if((Zigb_Rx_Buf[0] == 0x55) && (Zigb_Rx_Buf[1] == 0x0D))
			{
				if((Zigb_Rx_Buf[2] == 0x03) && (Zigb_Rx_Buf[3] == 0x01))
				{
					if(Zigb_Rx_Buf[4] == Floor)
					{
						break;
					}
				}
			}
		}
	}
}

/**********************************************************************
 * 函 数 名 ：  ZigBee通讯自定义发送函数
 * 参    数 ：  Head -> 包头
				Pri  -> 主指令
				Sec1,Sec2,Sec3	-> 副职令
 * 返 回 值 ：  无
 * 简    例 ：  Test_Zigbee(0x03,0x01,0x01,0x00,0x00);
**********************************************************************/
void Test_Zigbee(uint8_t Head,uint8_t Pri,uint8_t Sec1,uint8_t Sec2,uint8_t Sec3)
{
	Zigbee[0] = 0x55;
	Zigbee[1] = Head;
	Zigbee[2] = Pri;
	Zigbee[3] = Sec1;
	Zigbee[4] = Sec2;
	Zigbee[5] = Sec3;
	Zigbee[6] = (Zigbee[2]+Zigbee[3]+Zigbee[4]+Zigbee[5])%256;
	Zigbee[7] = 0xBB;
	Send_ZigbeeData_To_Fifo(Zigbee, 8);
	delay_ms(100);
}

/*****************************************************************
 * 函 数 名 ：	自动调节光照强度函数
 * 参    数 ：	gear -> 目标挡位值
 * 返 回 值 ：	gear_init -> 初始挡位值
 * 简     例：	gear_init = Light_Inf(3);	// 智能路灯调至3档
*****************************************************************/
uint8_t Light_Inf(uint8_t gear)
{
	uint8_t i;
	uint8_t gear_init = 0;	// 初始挡位值
	uint16_t array[2];		//缓存自学习的光档位数组
	
	if((gear > 0) && (gear < 5))
	{
		delay_ms(100);
		array[0] = Get_Bh_Value();		//光强度传感器	
		for(i=0; i<4; i++)
		{
			gear_init++;
			Infrared_Send(Light_plus1,4);
			delay_ms(500);
			delay_ms(500);
			delay_ms(500);
			array[1] = Get_Bh_Value();		//光强度传感器
			if (array[0] < array[1]) 
			{
				array[0] = array[1];
				array[1] = 0;
			}
			else
			{
				gear_init = 5 - gear_init;
				break;
			}
		}
		if(gear==2)
		{
			Infrared_Send(Light_plus1,4);	//光源档位加1
		}else if(gear==3)
		{
			Infrared_Send(Light_plus2,4);	//光源档位加2
		}else if(gear==4)
		{
			Infrared_Send(Light_plus3,4);	//光源档位加3
		}
	}
	return gear_init;
}

/*****************************************************************
 * 函 数 名 ：	ETC系统检测
 * 参    数 ：	无
 * 返 回 值 ：	无
 * 简     例：	ETC_Get_Zigbee();	// ETC系统检测
*****************************************************************/
void ETC_Get_Zigbee(void)
{
	tim_a = 0;
	tim_b = 0;
	Stop_Flag = 0;
	while(Stop_Flag != 0x06)
	{
		tim_a++;
		delay_ms(1);
		if(tim_a > 800)
		{
			tim_a = 0;
			tim_b++;
			if(tim_b < 6)
			{
				Car_Back(40, 100);	// 后退
				delay_ms(100);
				Car_Go(40, 110);	//小车前进
			}
			else
			{
				SYN_TTS("ETC系统识别失败，强制通行");
				break;
			}
		}
		
		if(Zigbee_Rx_flag == 1)	 // zigbee返回
		{
			Zigbee_Rx_flag = 0;
			delay_us(5);
			if(Zigb_Rx_Buf[1] == 0x0C)		// ETC系统
			{
				if(Zigb_Rx_Buf[2] == 0x01)
				{
					if(Zigb_Rx_Buf[3] == 0x01)
					{
						Stop_Flag = Zigb_Rx_Buf[4];	  // ETC闸门开启成功
					}
				}
			}
		}
	}
}

/*********************************************************************************
 * 函 数 名 ：	主车向Android终端发送任务请求函数
 * 参    数 ：	Pri  -> 主指令
				Sec  -> 副职令
				详见附录1、附录2
 * 返 回 值 ：	无
 * 简    例 ：	Send_Android(0xA1, 0x01);		// 调用摄像头预设位1


附录1：主车向Android终端发送调用任务指令结构
--------------------------------------------------------
| 主指令 | 副指令 |   说明
|--------|--------|-------------------------------------
|  0xA0  |  0x00  | 启动全自动
|--------|--------|-------------------------------------
|  0xA1  |  0x01  | 调用摄像头预设为1
|        |  0x02  | 调用摄像头预设为2
|        |  0x03  | 调用摄像头预设为3
|        |  0x04  | 调用摄像头预设为4
|        |  0x05  | 调用摄像头预设为5
|--------|--------|-------------------------------------
|  0xA2  |  0x00  | 调用二维码识别
|--------|--------|-------------------------------------
|  0xA3  |  0x00  | 调用交通灯识别
|--------|--------|-------------------------------------
|  0xA4  |  0x00  | 调用车牌识别
|--------|--------|-------------------------------------
|  0xA4  |  0x00  | 调用图形识别
--------------------------------------------------------

附录2：Android终端向主车发送识别结果指令结构
-------------------------------------------------------------------------------
| 主指令 | 副指令[1] | 副指令[2] | 副指令[3] |		说明
|--------|-----------|-----------|-----------|---------------------------------
|  0xA2  |   0x00    |   0x00    |   0x00	 | 二维码识别成功
|--------|-----------|-----------|-----------|---------------------------------
|  0xA3  |   0x01    |   0x00    |   0x00	 | 交通灯识别成功（红色）
|        |   0x02    |   0x00    |   0x00	 | 交通灯识别成功（黄色）
|        |   0x03    |   0x00    |   0x00	 | 交通灯识别成功（绿色）
|--------|-----------|-----------|-----------|---------------------------------
|  0xA4  |   0xXX    |   0xXX    |   0xXX	 | 车牌识别成功 车牌前三位ASICC
|--------|-----------|-----------|-----------|---------------------------------
|  0xA5  |   0xXX    |   0xXX    |   0xXX	 | 车牌识别成功 车牌后三位ASICC
|--------|-----------|-----------|-----------|---------------------------------
|  0xA6  |   0xXX    |   0xXX    |   0xXX	 | 图形识别 
|        |（三角形） | （圆形）  | （矩形）  | 
-------------------------------------------------------------------------------
*********************************************************************************/
void Send_Android(uint8_t Pri,uint8_t Sec)
{
	Principal_Tab[0] = 0x55;
	Principal_Tab[1] = 0xAA;
	Principal_Tab[2] = Pri;
	Principal_Tab[3] = Sec;
	Send_WifiData_To_Fifo(Principal_Tab, 4);   // 通过Wifi上传主车数据
	UartA72_TxClear();
	UartA72_TxAddStr(Principal_Tab, 4);        // 通过串口上传主车数据
	UartA72_TxStart();
}


/**************************** 运行控制指令 ****************************/
void Car_Go(uint8_t speed, uint16_t temp)   // 主车前进 参数：速度/码盘
{
    Roadway_mp_syn();       // 码盘同步
    Stop_Flag = 0;          // 运行状态标志位
    Go_Flag = 1;            // 前进标志位
    wheel_L_Flag = 0;       // 左转标志位
    wheel_R_Flag = 0;       // 右转标志位
    wheel_Nav_Flag = 0;     // 码盘旋转标志位
    Back_Flag = 0;          // 后退标志位
    Track_Flag = 0;         // 循迹标志位
    temp_MP = temp;         // 码盘值
    Car_Spend = speed;      // 速度值
    Control(Car_Spend, Car_Spend);  // 电机驱动函数
    while(Stop_Flag != 0x03);       // 等待前进完成
}

void Car_Back(uint8_t speed, uint16_t temp) // 主车后退 参数：速度/码盘
{
    Roadway_mp_syn();       // 码盘同步
    Stop_Flag = 0;          // 运行状态标志位
    Go_Flag = 0;            // 前进标志位
    wheel_L_Flag = 0;       // 左转标志位
    wheel_R_Flag = 0;       // 右转标志位
    wheel_Nav_Flag = 0;     // 码盘旋转标志位
    Back_Flag = 1;          // 后退标志位
    Track_Flag = 0;         // 循迹标志位
    temp_MP = temp;         // 码盘值
    Car_Spend = speed;      // 速度值
    Control(-Car_Spend, -Car_Spend); // 电机驱动函数
    while(Stop_Flag != 0x03);       // 等待后退完成
}

void Car_Left(uint8_t speed)       // 主车左转 参数：速度
{
	delay_ms(100);
    Stop_Flag = 0;          // 运行状态标志位
    Go_Flag = 0;            // 前进标志位
    wheel_L_Flag = 1;       // 左转标志位
    wheel_R_Flag = 0;       // 右转标志位
    wheel_Nav_Flag = 0;     // 码盘旋转标志位
    Back_Flag = 0;          // 后退标志位
    Track_Flag = 0;         // 循迹标志位
    Car_Spend = speed;      // 速度值
    Control(-Car_Spend, Car_Spend); // 电机驱动函数
    while(Stop_Flag != 0x02);       // 等待左转完成
	delay_ms(100);
}

void Car_Right(uint8_t speed)       // 主车右转 参数：速度
{
    delay_ms(100);
    Stop_Flag = 0;          // 运行状态标志位
    Go_Flag = 0;            // 前进标志位
    wheel_L_Flag = 0;       // 左转标志位
    wheel_R_Flag = 1;       // 右转标志位
    wheel_Nav_Flag = 0;     // 码盘旋转标志位
    Back_Flag = 0;          // 后退标志位
    Track_Flag = 0;         // 循迹标志位
    Car_Spend = speed;      // 速度值
    Control(Car_Spend, -Car_Spend); // 电机驱动函数
    while(Stop_Flag != 0x02);       // 等待左转完成
    delay_ms(100);
}

void Car_Track(uint8_t speed)   // 主车循迹 参数：速度
{
    Stop_Flag = 0;          // 运行状态标志位
    Go_Flag = 0;            // 前进标志位
    wheel_L_Flag = 0;       // 左转标志位
    wheel_R_Flag = 0;       // 右转标志位
    wheel_Nav_Flag = 0;     // 码盘旋转标志位
    Back_Flag = 0;          // 后退标志位
    Track_Flag = 1;         // 循迹标志位
    Car_Spend = speed;      // 速度值
    Control(Car_Spend, Car_Spend);  // 电机驱动函数 *
    while(Stop_Flag != 0x01);       // 等待循迹完成
}

void Car_L45(int8_t speed, uint16_t times)		// 左旋转 参数：旋转时间
{
	delay_ms(100);
	Send_UpMotor(-speed ,speed);	
	delay_ms(times);
	Send_UpMotor(0 ,0);
	delay_ms(100);
}

void Car_R45(int8_t speed, uint16_t tims)		// 右旋转 参数：旋转时间
{
	delay_ms(100);
	Send_UpMotor(speed,-speed);		// 电机驱动函数
	delay_ms(tims);					// 延时作角度
	Send_UpMotor(0 ,0);				// 停车
	delay_ms(100);
}

void Car_Time_Track(uint8_t speed, uint16_t tims)		// 小车循迹 参数：速度，时间
{
	Stop_Flag = 0;          // 运行状态标志位
    Go_Flag = 0;            // 前进标志位
    wheel_L_Flag = 0;       // 左转标志位
    wheel_R_Flag = 0;       // 右转标志位
    wheel_Nav_Flag = 0;     // 码盘旋转标志位
    Back_Flag = 0;          // 后退标志位
    Track_Flag = 1;         // 循迹标志位
    Car_Spend = speed;      // 速度值
	Control(Car_Spend, Car_Spend);  // 电机驱动函数 *
	if(tims <= 790)
	{
		delay_ms(tims);
	}
	else
	{
		delay_ms(790);
		delay_ms(tims-790);
	}
	Roadway_Flag_clean();	// 清除标志位状态	
	Send_UpMotor(0,0);		// 停车
	delay_ms(100);
}

void CarThread_Go(uint8_t speed, uint16_t temp)   // 主车前进 参数：速度/码盘
{
    Roadway_mp_syn();       // 码盘同步
    Stop_Flag = 0;          // 运行状态标志位
    Go_Flag = 1;            // 前进标志位
    wheel_L_Flag = 0;       // 左转标志位
    wheel_R_Flag = 0;       // 右转标志位
    wheel_Nav_Flag = 0;     // 码盘旋转标志位
    Back_Flag = 0;          // 后退标志位
    Track_Flag = 0;         // 循迹标志位
    temp_MP = temp;         // 码盘值
    Car_Spend = speed;      // 速度值
    Control(Car_Spend, Car_Spend);  // 电机驱动函数
}

void CarThread_Track(uint8_t speed)   // 主车循迹 参数：速度
{
    Stop_Flag = 0;          // 运行状态标志位
    Go_Flag = 0;            // 前进标志位
    wheel_L_Flag = 0;       // 左转标志位
    wheel_R_Flag = 0;       // 右转标志位
    wheel_Nav_Flag = 0;     // 码盘旋转标志位
    Back_Flag = 0;          // 后退标志位
    Track_Flag = 1;         // 循迹标志位
    Car_Spend = speed;      // 速度值
    Control(Car_Spend, Car_Spend);  // 电机驱动函数 *
}


#include "agv.h"
#include "delay.h"
#include "canp_hostcom.h"
#include "data_channel.h"
#include "data_base.h"
#include "cba.h"
#include "drive.h"


static uint16_t AGV_Time = 0, AGV_next = 0;	// 时间标记
static uint8_t AGV_Zigbee[8]={0x55,0x02,0x00,0x00,0x00,0x00,0x00,0xBB};
static uint8_t time[10] = {"time:00s\n"};

void AGV_Data_Open(void)	// AGV开启数据传回
{
	AGV_next = 0;
	AGV_Time = 5000;
	AGV_Zigbee[2] = 0x80;
	AGV_Zigbee[3] = 0x01;
	AGV_Zigbee[4] = 0x00;
	AGV_Zigbee[5] = 0x00;
	AGV_Zigbee[6] = (AGV_Zigbee[2]+AGV_Zigbee[3]+AGV_Zigbee[4]+AGV_Zigbee[5])%256;
	while(1)
	{
		AGV_Time++;
		delay_ms(1);
		if(AGV_Time > 1000)
		{
			AGV_Time = 0;
			AGV_next++;
			Send_ZigbeeData_To_Fifo(AGV_Zigbee, 8);
			delay_ms(100);
			Send_InfoData_To_Fifo("AGV_Get_Open\n",14);	// 上传调试信息
			delay_ms(100);
			if (AGV_next > 10)
			{
				break;
			}
		}
		if(Zigbee_Rx_flag == 1)	 //zigbee返回信息
		{
			Zigbee_Rx_flag =0;
			if((Zigb_Rx_Buf[0] == 0x55) && (Zigb_Rx_Buf[1] == 0x02))
			{
				break;
			}
		}
	}
}

void AGV_Data_Stop(void)	// AGV停止数据回传
{
	AGV_Time = 0;
	AGV_Zigbee[2] = 0x80;
	AGV_Zigbee[3] = 0x00;
	AGV_Zigbee[4] = 0x00;
	AGV_Zigbee[5] = 0x00;
	AGV_Zigbee[6] = (AGV_Zigbee[2]+AGV_Zigbee[3]+AGV_Zigbee[4]+AGV_Zigbee[5])%256;
	Send_ZigbeeData_To_Fifo(AGV_Zigbee, 8);
	delay_ms(100);
	Send_InfoData_To_Fifo("AGV_Data_Stop\n",14);	// 上传调试信息
	delay_ms(100);
	while(1)
	{
		AGV_Time++;
		delay_ms(1);
		if(AGV_Time > 1500)
		{
			break;
		}
		if(Zigbee_Rx_flag == 1)	 //zigbee返回信息
		{
			Zigbee_Rx_flag =0;
			if((Zigb_Rx_Buf[0] == 0x55) && (Zigb_Rx_Buf[1] == 0x02))
			{
				if(AGV_Time > 500)
				{
					AGV_Time = 0;
					Send_ZigbeeData_To_Fifo(AGV_Zigbee, 8);
					delay_ms(100);
					Send_InfoData_To_Fifo("AGV_Data_Stop\n",14);	// 上传调试信息
					delay_ms(100);
				}
			}
		}
	}
}

void AGV_GO(uint8_t speed, uint16_t mp)	// AGV前进 参数：速度 码盘
{
	AGV_Time = 5000;
	AGV_Zigbee[2] = 0x02;
	AGV_Zigbee[3] = speed;
	AGV_Zigbee[4] = mp%256;
	AGV_Zigbee[5] = mp/256;
	AGV_Zigbee[6] = (AGV_Zigbee[2]+AGV_Zigbee[3]+AGV_Zigbee[4]+AGV_Zigbee[5])%256;
	while(1)
	{
		AGV_Time++;
		delay_ms(1);
		if(AGV_Time > 3000)
		{
			AGV_Time = 0;
			Send_ZigbeeData_To_Fifo(AGV_Zigbee, 8);
			delay_ms(100);
			Send_InfoData_To_Fifo("AGV_GO\n",8);	// 上传调试信息
			delay_ms(100);
		}
		if(Zigbee_Rx_flag == 1)	 //zigbee返回信息
		{
			Zigbee_Rx_flag =0;
			if((Zigb_Rx_Buf[0] == 0x55) && (Zigb_Rx_Buf[1] == 0x02))
			{
				if(Zigb_Rx_Buf[2] == 0x03)
				{
					break;
				}
			}
		}
	}
}

void AGV_Break(uint8_t speed, uint16_t mp)	// AGV后退 参数：速度 码盘 
{
	AGV_Time = 5000;
	AGV_Zigbee[2] = 0x03;
	AGV_Zigbee[3] = speed;
	AGV_Zigbee[4] = mp%256;
	AGV_Zigbee[5] = mp/256;
	AGV_Zigbee[6] = (AGV_Zigbee[2]+AGV_Zigbee[3]+AGV_Zigbee[4]+AGV_Zigbee[5])%256;
	while(1)
	{
		AGV_Time++;
		delay_ms(1);
		if(AGV_Time > 3000)
		{
			AGV_Time = 0;
			Send_ZigbeeData_To_Fifo(AGV_Zigbee, 8);
			delay_ms(100);
			Send_InfoData_To_Fifo("AGV_Break\n",11);	// 上传调试信息
			delay_ms(100);
		}
		if(Zigbee_Rx_flag == 1)	 //zigbee返回信息
		{
			Zigbee_Rx_flag =0;
			if((Zigb_Rx_Buf[0] == 0x55) && (Zigb_Rx_Buf[1] == 0x02))
			{
				if(Zigb_Rx_Buf[2] == 0x03)
				{
					break;
				}
			}
		}
	}
}

void AGV_Track(uint8_t speed)	// AGV循迹 参数：速度
{
	AGV_Time = 5000;
	AGV_Zigbee[2] = 0x06;
	AGV_Zigbee[3] = speed;
	AGV_Zigbee[4] = 0x00;
	AGV_Zigbee[5] = 0x00;
	AGV_Zigbee[6] = (AGV_Zigbee[2]+AGV_Zigbee[3]+AGV_Zigbee[4]+AGV_Zigbee[5])%256;
	while(1)
	{
		AGV_Time++;
		delay_ms(1);
		if(AGV_Time > 3000)
		{
			AGV_Time = 0;
			Send_ZigbeeData_To_Fifo(AGV_Zigbee, 8);
			delay_ms(100);
			Send_InfoData_To_Fifo("AGV_Track\n",11);	// 上传调试信息
			delay_ms(100);
		}
		if(Zigbee_Rx_flag == 1)	 //zigbee返回信息
		{
			Zigbee_Rx_flag =0;
			if((Zigb_Rx_Buf[0] == 0x55) && (Zigb_Rx_Buf[1] == 0x02))
			{
				if(Zigb_Rx_Buf[2] == 0x01)
				{
					break;
				}
			}
		}
	}
}

void AGV_Reght(uint8_t speed)	// AGV右转 参数：速度
{
	AGV_Time = 5000;
	AGV_Zigbee[2] = 0x05;
	AGV_Zigbee[3] = speed;
	AGV_Zigbee[4] = 0x00;
	AGV_Zigbee[5] = 0x00;
	AGV_Zigbee[6] = (AGV_Zigbee[2]+AGV_Zigbee[3]+AGV_Zigbee[4]+AGV_Zigbee[5])%256;
	while(1)
	{
		AGV_Time++;
		delay_ms(1);
		if(AGV_Time > 3000)
		{
			AGV_Time = 0;
			Send_ZigbeeData_To_Fifo(AGV_Zigbee, 8);
			delay_ms(100);
			Send_InfoData_To_Fifo("AGV_Reght\n",11);	// 上传调试信息
			delay_ms(100);
		}
		if(Zigbee_Rx_flag == 1)	 //zigbee返回信息
		{
			Zigbee_Rx_flag = 0;
			if((Zigb_Rx_Buf[0] == 0x55) && (Zigb_Rx_Buf[1] == 0x02))
			{
				if(Zigb_Rx_Buf[2] == 0x02)
				{
					break;
				}
			}
		}
	}
}

void AGV_Left(uint8_t speed)		// AGV左转 参数：速度
{
	AGV_Time = 5000;
	AGV_Zigbee[2] = 0x04;
	AGV_Zigbee[3] = speed;
	AGV_Zigbee[4] = 0x00;
	AGV_Zigbee[5] = 0x00;
	AGV_Zigbee[6] = (AGV_Zigbee[2]+AGV_Zigbee[3]+AGV_Zigbee[4]+AGV_Zigbee[5])%256;
	while(1)
	{
		AGV_Time++;
		delay_ms(1);
		if(AGV_Time > 3000)
		{
			AGV_Time = 0;
			Send_ZigbeeData_To_Fifo(AGV_Zigbee, 8);
			delay_ms(100);
			Send_InfoData_To_Fifo("AGV_Left\n",10);	// 上传调试信息
			delay_ms(100);
		}
		if(Zigbee_Rx_flag == 1)	 //zigbee返回信息
		{
			Zigbee_Rx_flag =0;
			if((Zigb_Rx_Buf[0] == 0x55) && (Zigb_Rx_Buf[1] == 0x02))
			{
				if(Zigb_Rx_Buf[2] == 0x02)
				{
					break;
				}
			}
		}
	}
}

void AGV_STOP(void)		// AGV停止
{
	AGV_Time = 5000;
	AGV_Zigbee[2] = 0x01;
	AGV_Zigbee[3] = 0x00;
	AGV_Zigbee[4] = 0x00;
	AGV_Zigbee[5] = 0x00;
	AGV_Zigbee[6] = (AGV_Zigbee[2]+AGV_Zigbee[3]+AGV_Zigbee[4]+AGV_Zigbee[5])%256;
	while(1)
	{
		AGV_Time++;
		delay_ms(1);
		if(AGV_Time > 1000)
		{
			AGV_Time = 0;
			Send_ZigbeeData_To_Fifo(AGV_Zigbee, 8);
			delay_ms(100);
			Send_InfoData_To_Fifo("AGV_STOP\n",10);	// 上传调试信息
			delay_ms(100);
		}
		if(Zigbee_Rx_flag == 1)	 //zigbee返回信息
		{
			Zigbee_Rx_flag =0;
			if((Zigb_Rx_Buf[0] == 0x55) && (Zigb_Rx_Buf[1] == 0x02))
			{
				if(Zigb_Rx_Buf[2] == 0x00)
				{
					break;
				}
			}
		}
	}
}

void AGV_Beep(uint8_t mode)		// AGV_Zigbee开启蜂鸣器 参数：开关
{
	AGV_Zigbee[2] = 0x30;
	AGV_Zigbee[3] = mode;
	AGV_Zigbee[4] = 0x00;
	AGV_Zigbee[5] = 0x00;
	AGV_Zigbee[6] = (AGV_Zigbee[2]+AGV_Zigbee[3]+AGV_Zigbee[4]+AGV_Zigbee[5])%256;
	Send_ZigbeeData_To_Fifo(AGV_Zigbee, 8);
	delay_ms(100);
	Send_InfoData_To_Fifo("AGV_Beep\n",10);	// 上传调试信息
	delay_ms(10);
}

void AGV_Infrared(uint8_t *Src)		// AGV发送红外
{
	AGV_Zigbee[2] = 0x10;
	AGV_Zigbee[3] = Src[0];
	AGV_Zigbee[4] = Src[1];
	AGV_Zigbee[5] = Src[2];
	AGV_Zigbee[6] = (AGV_Zigbee[2]+AGV_Zigbee[3]+AGV_Zigbee[4]+AGV_Zigbee[5])%256;
	Send_ZigbeeData_To_Fifo(AGV_Zigbee, 8);
	delay_ms(500);
	
	AGV_Zigbee[2] = 0x11;
	AGV_Zigbee[3] = Src[3];
	AGV_Zigbee[4] = Src[4];
	AGV_Zigbee[5] = Src[5];
	AGV_Zigbee[6] = (AGV_Zigbee[2]+AGV_Zigbee[3]+AGV_Zigbee[4]+AGV_Zigbee[5])%256;
	Send_ZigbeeData_To_Fifo(AGV_Zigbee, 8);
	delay_ms(500);
	
	AGV_Zigbee[2] = 0x12;
	AGV_Zigbee[3] = 0x00;
	AGV_Zigbee[4] = 0x00;
	AGV_Zigbee[5] = 0x00;
	AGV_Zigbee[6] = (AGV_Zigbee[2]+AGV_Zigbee[3]+AGV_Zigbee[4]+AGV_Zigbee[5])%256;
	Send_ZigbeeData_To_Fifo(AGV_Zigbee, 8);
	delay_ms(100);
	Send_InfoData_To_Fifo("AGV_Infrared\n",14);	// 上传调试信息
	delay_ms(500);
}

void AGV_Wheel_L45(uint8_t speed,uint16_t time)		// AGV左转45 参数：速度
{
	AGV_Time = 5000;
	AGV_Zigbee[2] = 0xA1;
	AGV_Zigbee[3] = speed;
	AGV_Zigbee[4] = time%256;
	AGV_Zigbee[5] = time/256;
	AGV_Zigbee[6] = (AGV_Zigbee[2]+AGV_Zigbee[3]+AGV_Zigbee[4]+AGV_Zigbee[5])%256;
	while(1)
	{
		AGV_Time++;
		delay_ms(1);
		if(AGV_Time > 3000)
		{
			AGV_Time = 0;
			Send_ZigbeeData_To_Fifo(AGV_Zigbee, 8);
			delay_ms(100);
			Send_InfoData_To_Fifo("AGV_L45\n",9);	// 上传调试信息
			delay_ms(100);
		}
		if(Zigbee_Rx_flag == 1)	 //zigbee返回信息
		{
			Zigbee_Rx_flag =0;
			if((Zigb_Rx_Buf[0] == 0x55) && (Zigb_Rx_Buf[1] == 0x02))
			{
				if(Zigb_Rx_Buf[2] == 0x10)
				{
					break;
				}
			}
		}
	}
}

void AGV_Wheel_R45(uint8_t speed,uint16_t time)		// AGV右转45 参数：速度
{
	AGV_Time = 5000;
	AGV_Zigbee[2] = 0xA2;
	AGV_Zigbee[3] = speed;
	AGV_Zigbee[4] = time%256;
	AGV_Zigbee[5] = time/256;
	AGV_Zigbee[6] = (AGV_Zigbee[2]+AGV_Zigbee[3]+AGV_Zigbee[4]+AGV_Zigbee[5])%256;
	while(1)
	{
		AGV_Time++;
		delay_ms(1);
		if(AGV_Time > 3000)
		{
			AGV_Time = 0;
			Send_ZigbeeData_To_Fifo(AGV_Zigbee, 8);
			delay_ms(100);
			Send_InfoData_To_Fifo("AGV_R45\n",9);	// 上传调试信息
			delay_ms(100);
		}
		if(Zigbee_Rx_flag == 1)	 //zigbee返回信息
		{
			Zigbee_Rx_flag =0;
			if((Zigb_Rx_Buf[0] == 0x55) && (Zigb_Rx_Buf[1] == 0x02))
			{
				if(Zigb_Rx_Buf[2] == 0x11)
				{
					break;
				}
			}
		}
	}
}

void AGV_Thread(uint8_t mode)		// AGV自动驾驶
{
	AGV_next = 0;
	AGV_Time = 60000;
	AGV_Zigbee[2] = 0xA0;
	AGV_Zigbee[3] = mode;
	AGV_Zigbee[4] = 0x00;
	AGV_Zigbee[5] = 0x00;
	AGV_Zigbee[6] = (AGV_Zigbee[2]+AGV_Zigbee[3]+AGV_Zigbee[4]+AGV_Zigbee[5])%256;
	while(1)
	{
		AGV_Time++;
		delay_ms(10);
		if(AGV_Time > 5000)
		{
			AGV_Time = 0;
			AGV_next++;
			Send_ZigbeeData_To_Fifo(AGV_Zigbee, 8);
			delay_ms(100);
			Send_ZigbeeData_To_Fifo(AGV_Zigbee, 8);
			delay_ms(100);
			Send_InfoData_To_Fifo("AGV_Thread\n",12);	// 上传调试信息
			delay_ms(100);
			if (AGV_next > 3)
			{
				break;
			}
		}
		if(Zigbee_Rx_flag == 1)	 //zigbee返回信息
		{
			Zigbee_Rx_flag =0;
			if((Zigb_Rx_Buf[0] == 0x55) && (Zigb_Rx_Buf[1] == 0x02))
			{
				if(Zigb_Rx_Buf[2] == 0x08)
				{
					time[5] = AGV_Time/1000%10+0x30;
					time[6] = AGV_Time/100%10+0x30;
					Send_InfoData_To_Fifo((char *)time, 10);	// 上传调试信息
					break;
				}
			}
		}
	}
}

void AGV_GetThread(uint8_t mode)		// AGV全自动完成标志获取
{
	AGV_Time = 0;
	while(1)
	{
		AGV_Time++;
		delay_ms(1);
		if(AGV_Time > 60000)
		{
			AGV_Time = 0;
			break;
		}
		if(Zigbee_Rx_flag == 1)	 //zigbee返回信息
		{
			Zigbee_Rx_flag =0;
			if((Zigb_Rx_Buf[0] == 0x55) && (Zigb_Rx_Buf[1] == 0x02))
			{
				if(Zigb_Rx_Buf[2] == mode)
				{
					time[5] = AGV_Time/10000%10+0x30;
					time[6] = AGV_Time/1000%10+0x30;
					Send_InfoData_To_Fifo((char *)time, 10);	// 上传调试信息
					break;
				}
			}
		}
	}
}

void AGV_SendData(uint8_t Pri,uint8_t Sec1,uint8_t Sec2,uint8_t Sec3)	// AGV发送数据
{
	AGV_Zigbee[2] = Pri;
	AGV_Zigbee[3] = Sec1;
	AGV_Zigbee[4] = Sec2;
	AGV_Zigbee[5] = Sec3;
	AGV_Zigbee[6] = (AGV_Zigbee[2]+AGV_Zigbee[3]+AGV_Zigbee[4]+AGV_Zigbee[5])%256;
	Send_ZigbeeData_To_Fifo(AGV_Zigbee, 8);
	delay_ms(100);
	Send_InfoData_To_Fifo("AGV_Swit\n",10);	// 上传调试信息
	delay_ms(10);
}

/*
uint8_t AGV_GO_sp = 50;				// 小车前进速度
uint8_t AGV_wheel_sp = 90;			// 小车转弯速度
uint16_t AGV_GO_mp = 450;			// 小车前进码盘
	
void AGV_Thread(void)				// AGV全自动线程
{
	AGV_Data_Open();				// AGV开启数据上传
	AGV_Track(AGV_GO_sp);			// AGV循迹
	AGV_GO(AGV_GO_sp, AGV_GO_mp);	// AGV前进
	AGV_Track(AGV_GO_sp);			// AGV循迹
	AGV_GO(AGV_GO_sp, AGV_GO_mp);	// AGV前进
	
	AGV_Wheel_R45(AGV_wheel_sp, 450);
	delay_ms(500);
	AGV_Infrared(HW_K);				// AGV开启烽火台
	AGV_Infrared(HW_K);				// AGV开启烽火台
	delay_ms(500);
	AGV_Wheel_L45(AGV_wheel_sp, 450);
	
	AGV_Track(AGV_GO_sp);			// AGV循迹
	AGV_GO(AGV_GO_sp, AGV_GO_mp);	// AGV前进
	AGV_Reght(AGV_wheel_sp);		// AGV右转
	
	AGV_Track(AGV_GO_sp);			// AGV循迹
	AGV_GO(AGV_GO_sp, AGV_GO_mp);	// AGV前进
	AGV_Reght(AGV_wheel_sp);		// AGV右转
	
	AGV_Track(AGV_GO_sp);			// AGV循迹
	AGV_GO(AGV_GO_sp, AGV_GO_mp);	// AGV前进
	AGV_Track(AGV_GO_sp);			// AGV循迹
	AGV_GO(AGV_GO_sp, AGV_GO_mp);	// AGV前进
	AGV_Track(AGV_GO_sp);			// AGV循迹
	AGV_GO(AGV_GO_sp, 200);			// AGV前进
	AGV_Data_Stop();				// AGV关闭数据上传
}

*/




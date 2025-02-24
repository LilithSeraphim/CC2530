#include "stm32f4xx.h"
#include "CanP_Hostcom.h"
#include "delay.h"
#include "roadway_check.h"
#include "cba.h"
#include "Timer.h"

uint8_t wheel_L_Flag =0,wheel_R_Flag = 0;
uint8_t wheel_Nav_Flag = 0;
uint8_t Go_Flag = 0,Back_Flag = 0;
uint8_t Track_Flag = 0;

uint8_t Line_Flag = 0;
uint16_t count = 0;

uint8_t Stop_Flag = 0;
int LSpeed = 0,RSpeed = 0;
int Car_Spend = 0;
uint16_t temp_MP = 0;
uint16_t temp_Nav = 0;

uint32_t Wheel_flag = 0;

void Track_Correct(uint8_t gd);
void Back_Track(uint8_t gd);

//_________________________________________________________
int16_t Roadway_cmp;
extern int16_t CanHost_Mp;

/*
	码盘同步
**/
void Roadway_mp_syn(void)
{
	Roadway_cmp = CanHost_Mp;
}

/*
	码盘获取
**/
uint16_t Roadway_mp_Get(void)
{
	uint32_t ct;
	if(CanHost_Mp > Roadway_cmp)
		ct = CanHost_Mp - Roadway_cmp;
	else
		ct = Roadway_cmp - CanHost_Mp;
	if(ct > 0x8000)
		ct = 0xffff - ct;
	return ct;	
}

uint16_t Roadway_Navig;
extern uint16_t CanHost_Navig;

/*
	角度同步
**/
void Roadway_nav_syn(void)
{
	Roadway_Navig = CanHost_Navig;
}

/*
	获取角度差值
**/
uint16_t Roadway_nav_Get(void)
{
	uint16_t ct;
	if(CanHost_Navig > Roadway_Navig)
		ct = CanHost_Navig - Roadway_Navig;
	else
		ct = Roadway_Navig - CanHost_Navig;
	while(ct >= 36000)
		ct -= 36000;
	return ct;
}

//_______________________________________________________________

void Roadway_Flag_clean(void)
{
	wheel_L_Flag =0;wheel_R_Flag = 0;
	Go_Flag = 0;Back_Flag = 0;
	Track_Flag = 0;
	Stop_Flag = 0;
//	temp_MP = 0;
}

/**
	前进监测
*/
void Go_and_Back_Check(void)
{	
	if(Go_Flag == 1)
	{
		if(temp_MP <= Roadway_mp_Get())
		{
			Go_Flag = 0;
			Stop_Flag = 3;
			Send_UpMotor(0,0);		// 停止
		}
	} 
	else if(Back_Flag == 1)
	{
		if(temp_MP <= Roadway_mp_Get())
		{
			Back_Flag = 0;
			Stop_Flag=3;
			Send_UpMotor(0,0);
		}
	}
}

uint8_t Roadway_GoBack_Check(void)
{
	return ((Go_Flag == 0)&&(Back_Flag == 0)&&(Track_Flag == 0)&&(wheel_L_Flag == 0)&&(wheel_R_Flag == 0))? 1:0;
}

/**
	码盘转弯
*/
void wheel_Nav_check(void)
{ 	
	uint16_t Mp_Value = 0;
	
	if(wheel_Nav_Flag)
	{
		Mp_Value = Roadway_mp_Get(); 
		if(Mp_Value >= temp_Nav)
		{
			wheel_Nav_Flag = 0;
			Stop_Flag = 2;
			Send_UpMotor(0,0);
		}
	}
}


/**
	根据循迹线转弯
*/
uint32_t Mp_Value = 0;
void wheel_Track_check(void)
{ 	
	uint16_t Track_Value = 0;
	
	if(wheel_L_Flag == 1)
	{
		Track_Value = Get_Host_UpTrack(TRACK_H8);
		if(!(Track_Value & 0X10))		// 找到循迹线，停止
		{	
			if(Wheel_flag > 50)
			{
				wheel_L_Flag = 0;
				Wheel_flag=0;
				Stop_Flag=2;
				Send_UpMotor(0,0);
			}
		}
		else if(Track_Value == 0Xff)	// 循迹灯全亮
		{			
			Wheel_flag++;
		}
	} 
	else if(wheel_R_Flag == 1)
	{
		Track_Value = Get_Host_UpTrack(TRACK_H8);

		 if(!(Track_Value &0X08)) //找到循迹线，停止
			{	
				if(Wheel_flag > 50)
				{
					wheel_R_Flag=0;
					Wheel_flag=0;
					Stop_Flag=2;
					Send_UpMotor(0,0);
				}
			}
			else if( Track_Value == 0Xff) 
			{				
				Wheel_flag++;
			}
	}
}


/**
	循迹监测
*/
void Track_Check()
{	
	if(Track_Flag == 1)
	{
		Track_Correct(Get_Host_UpTrack(TRACK_H8));
	}
}

void Roadway_Check(void)
{
	Go_and_Back_Check();
	wheel_Track_check();
	wheel_Nav_check();
	Track_Check();
	
}
	

/***************************************************************
** 功能：     电机控制函数
** 参数：	  L_Spend：电机左轮速度
**            R_Spend：电机右轮速度
** 返回值：   无	  
****************************************************************/
void Control(int L_Spend,int R_Spend)
{
	if(L_Spend>=0)
	{	
		if(L_Spend>100)L_Spend=100;if(L_Spend<5)L_Spend=5;		//限制速度参数
	}
	else 
	{
		if(L_Spend<-100)L_Spend= -100;if(L_Spend>-5)L_Spend= -5;     //限制速度参数
	}	
	if(R_Spend>=0)
	{	
		if(R_Spend>100)R_Spend=100;if(R_Spend<5)R_Spend=5;		//限制速度参数
	}
	else
	{	
		if(R_Spend<-100)R_Spend= -100;if(R_Spend>-5)R_Spend= -5;		//限制速度参数		
	}
	Send_UpMotor(L_Spend ,R_Spend);	
}

extern uint8_t RFID_Flag;
extern uint8_t Terrain_Flag;
/***************************************************************
** 功能：     循迹函数
** 参数：	  无参数
** 返回值：   无
****************************************************************/
void Track_Correct(uint8_t gd)
{
	if(gd == 0x00)	// 循迹灯全灭 停止
	{
		Track_Flag = 0;
		Stop_Flag = 1;
		Send_UpMotor(0,0);
		
	} else if(gd==0xE7)
	{
		LSpeed=Car_Spend;
		RSpeed=Car_Spend;
	} else if((gd==0xF7) || (gd==0XF3))
	{
		LSpeed=Car_Spend+20;
		RSpeed=Car_Spend-40;
	} else if((gd==0XFB) || (gd==0XF9))
	{
		LSpeed=Car_Spend+40;
		RSpeed=Car_Spend-60;
	} else if((gd==0XFD) || (gd==0XFC))
	{
		LSpeed=Car_Spend+60;
		RSpeed=Car_Spend-90;
	} else if(gd==0XFE)
	{
		LSpeed=Car_Spend+80;
		RSpeed=Car_Spend-120;
	} else if((gd==0XEF) || (gd==0XCF))
	{
		RSpeed = Car_Spend+20;
		LSpeed = Car_Spend-40;
	} else if((gd==0XDF) || (gd==0X9F))
	{
		RSpeed = Car_Spend+40;
		LSpeed = Car_Spend-60;
	} else if((gd==0XBF) || (gd==0X3F))
	{
		RSpeed = Car_Spend+60;
		LSpeed = Car_Spend-90;
	} else if(gd==0X7F)
	{
		RSpeed = Car_Spend+80;
		LSpeed = Car_Spend-120;
	} else if(RFID_Flag == 1)
	{
		if(((gd&0x18)==0x18)&&(((gd&0x80)==0x00)||((gd&0x01)==0x00)))
		{
			Track_Flag = 0;
			Stop_Flag = 1;
			Send_UpMotor(0,0);
		}
		
	} else if(Terrain_Flag == 1)
	{
		if(gd==0x18)
		{
			LSpeed=Car_Spend;
			RSpeed=Car_Spend;
		} else if((gd==0x08) || (gd==0x0C))
		{
			LSpeed=Car_Spend+20;
			RSpeed=Car_Spend-40;
		} else if((gd==0x04) || (gd==0x06))
		{
			LSpeed=Car_Spend+40;
			RSpeed=Car_Spend-60;
		} else if((gd==0x02) || (gd==0x03))
		{
			LSpeed=Car_Spend+60;
			RSpeed=Car_Spend-90;
		} else if(gd==0x01)
		{
			LSpeed=Car_Spend+80;
			RSpeed=Car_Spend-120;
		} else if((gd==0x10) || (gd==0x30))
		{
			RSpeed = Car_Spend+20;
			LSpeed = Car_Spend-40;
		} else if((gd==0x20) || (gd==0x60))
		{
			RSpeed = Car_Spend+40;
			LSpeed = Car_Spend-60;
		} else if((gd==0x40) || (gd==0xC0))
		{
			RSpeed = Car_Spend+60;
			LSpeed = Car_Spend-90;
		} else if(gd==0X80)
		{
			RSpeed = Car_Spend+80;
			LSpeed = Car_Spend-120;
		} else 
		{
			LSpeed = Car_Spend;
			RSpeed = Car_Spend;
		}
	} else 
	{
		LSpeed = Car_Spend;
		RSpeed = Car_Spend;
	}
	
	if(gd==0xFF)   //循迹灯全亮
	{
		LSpeed = Car_Spend;
		RSpeed = Car_Spend;
		if(count > 1200)
		{
			count=0;
			Send_UpMotor(0,0);
			Track_Flag=0;
			Stop_Flag = 4;
		}
		else {
			count++;
		}			
	}
	else 
	{
		count=0;
	}
		
	if(Track_Flag != 0)
	{
		Control(LSpeed,RSpeed);
	}
}

/***************************************************************
** 功能：     循迹函数
** 参数：	  无参数
** 返回值：   无
****************************************************************/
void Track_Roadway(uint8_t gd)
{
	if(gd == 0x00)	// 循迹灯全灭 停止
	{
		Track_Flag = 0;
		Stop_Flag = 1;
		Send_UpMotor(0,0);
		
	} else if(gd==0x18)
	{
		LSpeed=Car_Spend;
		RSpeed=Car_Spend;
		
	} else if((gd==0x08) || (gd==0x0C))
	{
		LSpeed=Car_Spend+20;
		RSpeed=Car_Spend-40;
	} else if((gd==0x04) || (gd==0x06))
	{
		LSpeed=Car_Spend+40;
		RSpeed=Car_Spend-60;
	} else if((gd==0x02) || (gd==0x03))
	{
		LSpeed=Car_Spend+60;
		RSpeed=Car_Spend-90;
		
	} else if(gd==0x01)
	{
		LSpeed=Car_Spend+80;
		RSpeed=Car_Spend-120;
		
	} else if((gd==0x10) || (gd==0x30))
	{
		RSpeed = Car_Spend+20;
		LSpeed = Car_Spend-40;
		
	} else if((gd==0x20) || (gd==0x60))
	{
		RSpeed = Car_Spend+40;
		LSpeed = Car_Spend-60;
		
	} else if((gd==0x40) || (gd==0xC0))
	{
		RSpeed = Car_Spend+60;
		LSpeed = Car_Spend-90;
		
	} else if(gd==0X80)
	{
		RSpeed = Car_Spend+80;
		LSpeed = Car_Spend-120;
		
	} else 
	{
		LSpeed=Car_Spend;
		RSpeed=Car_Spend;
	}
	
	if(gd==0xFF)   //循迹灯全亮
	{
		if(count > 1000)
		{
			count=0;
			Send_UpMotor(0,0);
			Track_Flag=0;
			Stop_Flag = 4;
		}
		else {
			count++;
		}			
	}
	else 
	{
		count=0;
	}
		
	if(Track_Flag != 0)
	{
		Control(LSpeed,RSpeed);
	}
}

void roadway_check_TimInit(uint16_t arr,uint16_t psc)
{
	TIM_TimeBaseInitTypeDef TIM_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,ENABLE);
	
	TIM_InitStructure.TIM_Period = arr;
	TIM_InitStructure.TIM_Prescaler = psc;
	TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_InitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM9,&TIM_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_TIM9_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_ITConfig(TIM9,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM9, ENABLE);
}

void TIM1_BRK_TIM9_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM9,TIM_IT_Update) == SET)
	{
		Roadway_Check();		// 路况检测
	}
	TIM_ClearITPendingBit(TIM9,TIM_IT_Update);
}


//end file

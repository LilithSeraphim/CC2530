/**
 * 2019年综合例程
 * 修改时间：2019.05.12
 */
#include <stdio.h>
//#include <string.h>
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
#include "drive.h"
#include "agv.h"
//饼图
uint8_t b_tu[6]={0xFF,0x12,0x06,0x00,0x00,0x00};
//自定义
uint8_t mydi[13]={0xFD,0x00,0x0A,0x53,0x01,0xD4,0xAD,0xC9,0xF1,0xC6,0xF4,0xB6,0xAF};
//数组
uint8_t mydef[69]={0xFD,0x00,0x42,0x1F,0x01,0x33,0xC6,0xEB,0xCD,0xB7,0xB2,0xA2,0xBD,0xF8,0x20,0x31,0x31,0x30,0x7C,0xC3,0xC0,0xBA,0xC3,0xC9,0xFA,0xBB,0xEE,0x20,0x31,0x31,0x31,0x7C,0xD0,0xE3,0xC0,0xF6,0xC9,0xBD,0xBA,0xD3,0x20,0x31,0x31,0x32,0x7C,0xD7,0xB7,0xD6,0xF0,0xC3,0xCE,0xCF,0xEB,0x20,0x31,0x31,0x33,0x7C,0x7C,0x7C,0x7C,0x7C,0x7C,0x7C,0x7C,0x7C,0x7C,0x7C,0x7C};
//函数声明
void SYN7318_Put_String(uint8_t* Pst,uint8_t Length);
/***************************** 自定义变量 *****************************/
static uint8_t Go_Speed  = 50;              // 全局前进速度值
static uint8_t wheel_Speed = 95;            // 全局转弯速度值
static uint16_t Go_Temp = 450;              // 全局转弯速度值

static uint32_t Power_check_times;          // 电量检测周期
static uint32_t LED_twinkle_times;          // LED闪烁周期
static uint32_t WIFI_Upload_data_times;     // 通过Wifi上传数据周期
static uint32_t RFID_Init_Check_times;      // RFID初始化检测时间周期

uint8_t make = 0;                   // 全自动驾驶标志位
uint8_t RFID_Flag = 0;          	// RFID检测标志位
uint8_t SYN7318_Flag = 0;           // SYN7318语音识别命令ID编号
uint8_t number = 0;                 // 计数值
uint8_t RFID_addr = 0;				// RFID有效数据块地址
uint8_t Terrain_Flag = 0;			// 地形检测标志位
uint16_t dis_size = 0;              // 超声波测距值缓存
uint16_t tim_i,tim_j;

uint8_t AGV_GO_sp = 50;				// 小车前进速度
uint8_t AGV_wheel_sp = 90;			// 小车转弯速度
uint16_t AGV_GO_mp = 420;			// 小车前进码盘

uint8_t SYN_Sesult[8] = {0xAF,0x06,0x00,0x02,0x00,0x00,0x01,0xBF};	// 语音识别编号上传协议

static uint8_t timestart[8] = {0x55,0x08,0x30,0x01,0x00,0x00,0x31,0xBB};
static uint8_t timefinish[8] = {0x55,0x08,0x30,0x00,0x00,0x00,0x30,0xBB};


static void Car_Thread(void);               // 全自动函数
static void KEY_Check(void);                // 按键检测函数
static void Hardware_Init(void);            // 硬件初始化函数
/**********************************************************************/

#if 0

/* 全自动运行函数 */
void Car_Thread(void)
{
    switch(make)
    {
        case 0x01:
        {
            Send_ZigbeeData_To_Fifo(SEG_TimOpen, 8);    // LED显示标志物计时模式 -> 开启
            delay_ms(200);
            Send_ZigbeeData_To_Fifo(SEG_TimOpen, 8);    // LED显示标志物计时模式 -> 开启
            delay_ms(300);
			
			TFT_Test_Zigbee('A',0x40,0x20,0x19,0x05);    // TFT显示日期
			delay_ms(200);
			TFT_Test_Zigbee('A',0x40,0x20,0x19,0x05);    // TFT显示日期
			delay_ms(100);
			
			Send_Android(0xA1, 0x01);		// 调用摄像头预设位1
			
            Car_Track(Go_Speed);            // 主车循迹
            Car_Go(Go_Speed, Go_Temp);      // 主车前进
			Car_Right(wheel_Speed);         // 主车右转
			Car_Time_Track(Go_Speed, 500);	// 主车时间循迹
			
//---------------------- 智能交通灯标志物 ----------------------
            delay_ms(500);
            Send_ZigbeeData_To_Fifo(TrafficA_Open, 8);   // 智能交通灯进入识别模式
            delay_ms(500); delay_ms(500); delay_ms(500);
            Send_ZigbeeData_To_Fifo(TrafficA_Yellow, 8);    // 智能交通灯 -> 黄色
            delay_ms(300);
			
            Car_Track(Go_Speed);            // 主车循迹
            Car_Go(Go_Speed, Go_Temp);		// 主车前进
			Car_Track(Go_Speed);            // 主车循迹

//------------------- 静态标志物 --------------------------
            delay_ms(500);
            Ultrasonic_Ranging();           // 采集超声波测距距离
            dis_size = dis;
            LED_Dis_Zigbee(dis_size);       // LED显示标志物发送测距信息
			LED_Dis_Zigbee(dis_size);       // LED显示标志物发送测距信息
			TFT_Dis_Zigbee('B',dis_size);	// TFT显示距离信息
			TFT_Dis_Zigbee('B',dis_size);	// TFT显示距离信息
			
			Car_Go(Go_Speed, Go_Temp+10);   // 主车前进
			Car_Left(wheel_Speed);          // 主车左转
			
//------------------- 立体显示 -------------------------
            Car_L45(90, 440);               // 旋转
            delay_ms(500);
//			Infrared_Send(Rotate_2,6);		// 立体显示"前方施工，请绕行"
			Rotate_Dis_Inf(dis_size);		// 向立体显示物发送测距信息
//            Rotate_show_Inf("A123B4", 'F', '2');	// 向立体显示物发送车牌信息
            delay_ms(100);
            Car_R45(90, 440);               // 旋转
			delay_ms(100);
			
			make = 0x02;
			break;
		}
		
		case 0x02:
		{
//------------------- 地形检测 ---------------------------
			Terrain_Flag = 1;
            Car_Track(Go_Speed);            // 主车循迹
			Terrain_Flag = 0;				// 清除地形检测标志位
			
			Car_Go(Go_Speed, Go_Temp+10);   // 主车前进
			Car_Right(wheel_Speed);         // 主车右转
			
//------------------- 智能路灯 ---------------------------
            delay_ms(500);
            Light_Inf(3);                   // 自动调节光照强度函数
			delay_ms(500);
			SYN_TTS("路灯已调[=tiao2]至3档[=dang3]");
			
			Car_Left(wheel_Speed);          // 主车左转
			Car_Left(wheel_Speed);          // 主车左转
			
//------------------- RFID ---------------------------
			RFID_addr = 40;					// RFID有效数据块地址
			RFID_Flag = 10;					// RFID检测标志位/行进速度降低参数
			CarThread_Track(Go_Speed - RFID_Flag); // 主车循迹
			make = 0x03;
			break;
		}
			
		case 0x03:
		{
			if (Stop_Flag == 0x01)
			{
				make = 0x04;
			}
			if ((RFID_Flag != 0) && (PcdRequest(PICC_REQALL,CT) == MI_OK))		// RFID寻卡
			{
				Roadway_Flag_clean();	// 清除标志位状态
				Send_UpMotor(0,0);		// 停车
				delay_ms(100);
				if (Read_RFID(RFID_addr) == 0x02)		// RFID 读卡 -> READ_RFID[16]
				{
					MP_SPK = 1;
					delay_ms(500);
					MP_SPK = 0;
					SYN_TTS("读卡成功");
					RFID_Flag = 0;				// 清除RFID标志位
					CarThread_Track(Go_Speed);	// 主车循迹
				}
				else
				{
					CarThread_Track(Go_Speed - RFID_Flag); // 主车循迹
				}
			}
			break;
		}
			
		case 0x04:
		{
			RFID_Flag = 0;					// 清除RFID标志位
			Car_Go(Go_Speed, Go_Temp);   	// 主车前进
			Car_Right(wheel_Speed);         // 主车右转
			Car_Time_Track(Go_Speed, 500);	// 主车时间循迹
			
//-------------------- 智能TFT显示器 --------------------
            delay_ms(500);
			delay_ms(500);
//          TFT_Test_Zigbee('A',0x10,0x01,0x00,0x00);   // TFT显示器图片向上翻页
			TFT_Test_Zigbee('A',0x10,0x02,0x00,0x00);   // TFT显示器图片向下翻页
			delay_ms(500);
			TFT_Test_Zigbee('A',0x10,0x02,0x00,0x00);   // TFT显示器图片向下翻页
			
			delay_ms(600); delay_ms(600); delay_ms(600); delay_ms(600);
			TFT_Test_Zigbee('A',0x10,0x02,0x00,0x00);   // TFT显示器图片向下翻页
			delay_ms(500);
			TFT_Test_Zigbee('A',0x10,0x02,0x00,0x00);   // TFT显示器图片向下翻页
			
			delay_ms(600); delay_ms(600); delay_ms(600); delay_ms(600); 
			TFT_Test_Zigbee('A',0x40,0x12,0x34,0x56);   // TFT显示器显示图形信息
			delay_ms(200);
			TFT_Test_Zigbee('A',0x40,0x12,0x34,0x56);   // TFT显示器显示图形信息
			delay_ms(100);
//			TFT_Show_Zigbee('A',"A88888");              // TFT显示器显示图形信息
			delay_ms(500);
			
            Car_Back(Go_Speed, 320);		// 后退
			Car_Left(wheel_Speed);          // 主车左转
#if 0
			Car_Time_Track(Go_Speed, 500);	// 时间循迹
			
//---------------------- 智能交通灯标志物 ----------------------
            delay_ms(100);
            Send_ZigbeeData_To_Fifo(TrafficB_Open, 8);   // 智能交通灯进入识别模式
            delay_ms(500);
            delay_ms(500);
            Send_ZigbeeData_To_Fifo(TrafficB_Yellow, 8);	// 智能交通灯 -> 黄色
            delay_ms(100);
#endif 
            Car_Track(Go_Speed);            // 主车循迹
			
//---------------------- 语音播报标志物 ----------------------
			number = 3;         // 语音识别次数
            do 
						{
                SYN7318_Flag = SYN7318_Extern();
            }
            while ((!SYN7318_Flag) && (--number));
						
						
            SYN_Sesult[2] = SYN7318_Flag;
            Send_ZigbeeData_To_Fifo(SYN_Sesult, 8);    // 上传语音命令编号
            delay_ms(100);
			
//------------------- 移动机器人全自动调用 --------------------
			SYN_TTS("小伙伴 出发");
			Send_ZigbeeData_To_Fifo(AGV_Thread1,8);		// 移动机器人全自动调用
			delay_ms(200);
			Send_ZigbeeData_To_Fifo(AGV_Thread1,8);		// 移动机器人全自动调用
			delay_ms(200);
			Send_ZigbeeData_To_Fifo(AGV_Thread1,8);		// 移动机器人全自动调用
			delay_ms(100);
			
//-------------------- 立体车库复位 ----------------------------
			Send_ZigbeeData_To_Fifo(GarageA_To1, 8);	// 车库到达第一层
			delay_ms(200);
			Send_ZigbeeData_To_Fifo(GarageA_To1, 8);	// 车库到达第一层
			delay_ms(200);
			
			Car_Go(Go_Speed, Go_Temp+10);   // 主车前进
			Car_Left(wheel_Speed);          // 主车左转
			
//-------------------- 道闸系统 --------------------
//			Gate_Show_Zigbee("A88888");		// 道闸显示车牌
			Send_ZigbeeData_To_Fifo(Gate_Open, 8);       // 道闸 -> 开启
			//0x55,0x03,0x01,0x01,0x00,0x00,0x02 ,0xbb
			
			delay_ms(200);
			Send_ZigbeeData_To_Fifo(Gate_Open, 8);       // 道闸 -> 开启
			delay_ms(500);
			delay_ms(500);
			
			Car_Track(Go_Speed);            // 主车循迹
			Car_Go(Go_Speed, Go_Temp);   	// 主车前进
			Car_Left(wheel_Speed);          // 主车左转
			Car_Time_Track(Go_Speed, 100);	// 时间循迹
			
//-------------------- ETC测试 ------------------------
			ETC_Get_Zigbee();				// ETC系统检测
			
			TFT_Show_Zigbee('A',"A88888");    // TFT显示器显示图形信息
			delay_ms(300);
			TFT_Show_Zigbee('A',"A88888");    // TFT显示器显示图形信息
			delay_ms(100);
			
			Car_Track(Go_Speed);            // 主车循迹
			Car_Go(Go_Speed, Go_Temp+10);   // 主车前进
			
//------------------- 烽火台 ---------------------------
			Car_L45(90, 400);               // 旋转
            delay_ms(500);
            Infrared_Send(Alarm_Open, 6);   // 烽火台 -> 开启
            delay_ms(200);
			Car_R45(90, 400);               // 旋转
			
			Car_Right(wheel_Speed);         // 主车右转
			Car_Track(Go_Speed);            // 主车循迹
			Car_Go(Go_Speed, Go_Temp);      // 主车前进
			Car_Left(wheel_Speed);          // 主车左转
			Car_Track(Go_Speed);            // 主车循迹
			
//------------------- 静态标志物 --------------------------
            delay_ms(500);
            Ultrasonic_Ranging();           // 采集超声波测距距离
            dis_size = dis;
            LED_Dis_Zigbee(dis_size);       // LED显示标志物发送测距信息
			LED_Dis_Zigbee(dis_size);       // LED显示标志物发送测距信息
			TFT_Dis_Zigbee('B',dis_size);	// TFT显示距离信息
			TFT_Dis_Zigbee('B',dis_size);	// TFT显示距离信息
			
			Car_Go(Go_Speed, Go_Temp-10);   	// 主车前进
			Car_Left(wheel_Speed);          // 主车左转
			Car_Time_Track(Go_Speed, 800);	// 时间循迹
			
//------------------- 立体车库A ---------------------------
			Garage_Cont_Zigbee('A',1);		// 立体车库A 到达第1层
			
			Car_Back(Go_Speed, 1900);		// 后退
			
			Garage_Cont_Zigbee('A',3);		// 立体车库A 到达第3层
			
            Send_ZigbeeData_To_Fifo(Charge_Open, 8);  // 无线充电 -> 开启
            delay_ms(200);
            Send_ZigbeeData_To_Fifo(Charge_Open, 8);  // 无线充电 -> 开启
            delay_ms(100);
			
//			AGV_Data_Open();				// AGV开启数据上传
//			AGV_GetThread(0xA1);			// AGV全自动完成标志获取
//			AGV_Data_Stop();				// AGV关闭数据上传
			
            delay_ms(100);
            Send_ZigbeeData_To_Fifo(SMG_TimClose, 8);  // 数码管计时 -> 关闭
            delay_ms(300);
            Send_ZigbeeData_To_Fifo(SMG_TimClose, 8);  // 数码管计时 -> 关闭
            delay_ms(500);

            Set_tba_Beep(SET);      // 任务板蜂鸣器 -> 开启
            delay_ms(500);
            delay_ms(500);
            delay_ms(500);
            Set_tba_Beep(RESET);    // 任务板蜂鸣器 -> 关闭
			
			make = 0x05;
			break;
		}
			
		case 0x05:
		{
//------------------- 立体车库A ---------------------------
			SYN_TTS("3秒后启动全自动");
			delay_ms(100);
			
//------------------- 沙盘复位 ---------------------------		
			TFT_Test_Zigbee('A',0x10,0x02,0x00,0x00);    // TFT显示器图片向下翻页
			delay_ms(500);
			TFT_Test_Zigbee('A',0x10,0x02,0x00,0x00);    // TFT显示器图片向下翻页
			delay_ms(100);
			TFT_Test_Zigbee('B',0x10,0x02,0x00,0x00);    // TFT显示器图片向下翻页
			delay_ms(500);
			TFT_Test_Zigbee('B',0x10,0x02,0x00,0x00);    // TFT显示器图片向下翻页
			delay_ms(100);
			
			Send_ZigbeeData_To_Fifo(SEG_TimOpen, 8);    // LED显示标志物计时模式 -> 开启
            delay_ms(300);
            Send_ZigbeeData_To_Fifo(SEG_TimOpen, 8);    // LED显示标志物计时模式 -> 开启
            delay_ms(300);
			
			Garage_Cont_Zigbee('A',1);		// 立体车库A 到达第1层
			delay_ms(100);
			
			Car_Go(Go_Speed, 500);      	// 主车前进
			Car_Track(Go_Speed);            // 主车循迹
            Car_Go(Go_Speed, Go_Temp);      // 主车前进
			
//------------------- 立体显示 -------------------------
            Car_L45(90, 450);               // 旋转
            delay_ms(500);
			Infrared_Send(Rotate_2,6);		// 立体显示"前方施工，请绕行"
            delay_ms(100);
            Car_R45(90, 450);               // 旋转
			delay_ms(100);
			
            make = 0x02;
			break;
        }
		
        case 0xA0:
        {
            make = 0;
        }
        break;

        case 0xA1:
		{
            make = 0;
            break;
		}

        case 0xA2:
		{
			make = 0;
            break;
		}

        case 0xA3:
		{
			make = 0;
            break;
		}
		
		default : 
			break;
    }
}


#endif

/* 按键检测函数 */
void KEY_Check(void)
{
	/*
    if(S1 == 0)
    {
        delay_ms(10);
        if(S1 == 0)
        {
            LED1 = !LED1;
            while(!S1);
            delay_ms(500);
            delay_ms(500);
            //make = 0x01;
			*/
		//泰

		if(S1 == 0)
    {
        delay_ms(10);
        if(S1 == 0)
        {
					LED1 = !LED1;
					
					
					TFT_Test_Zigbee('A',0x30,0x01,0x00,0x00);
					delay_ms(50);
					TFT_Test_Zigbee('A',0x30,0x01,0x00,0x00);
					delay_ms(50);
					Send_ZigbeeData_To_Fifo(timestart,8);
					delay_ms(50);
					
					Car_Track(Go_Speed);            // 主车循迹
          Car_Go(Go_Speed, Go_Temp);      // 主车前进
					Car_Left(wheel_Speed);         // 主车左转
					Car_Track(Go_Speed);            // 主车循迹
          Car_Go(Go_Speed, Go_Temp);      // 主车前进
					Car_Right(wheel_Speed);
					//道闸
					Send_ZigbeeData_To_Fifo(Gate_Open, 8); //开启闸门
					delay_ms(150);
					while(1)
					{
						Send_ZigbeeData_To_Fifo(Gate_GetStatus, 8);//查询状态
						delay_ms(20);
						Can_ZigBeeRx_Check(); 
						if(strcmp(Stop_Flag,0x05)==1)
								Send_UpMotor(0, 0);
						else if (strcmp(Stop_Flag,0x05)==0)
							break;
					}					
					Gate_Show_Zigbee("545233");//显示车牌
					
					
					
					Car_Track(Go_Speed);            // 主车循迹
          Car_Go(Go_Speed, Go_Temp);      // 主车前进	
					Send_ZigbeeData_To_Fifo(Gate_Close, 8);//关闭闸门					
					Car_Right(wheel_Speed);
					
					
					Car_Track(Go_Speed);            // 主车循迹
          Car_Go(Go_Speed, Go_Temp);      // 主车前进	
					Car_R45(90, 400);
					//语音播报
					
					number = 5;         // 语音识别次数
          do 
					{
               SYN7318_Flag = SYN7318_Extern();
          }
          while ((!SYN7318_Flag) && (--number));
						
					/*int number=5;
					do
					{
					//YY_Comm_Zigbee(0x10,0x06);//标志物播放语音原地掉头SYN_Sesult[1]
					SYN7318_Extern();
					if(SYN7318_Flag)
						break;
					}while (--number);*/
					
					delay_ms(50);
					Send_ZigbeeData_To_Fifo(GarageA_To1, 8);
					delay_ms(50);
					Send_ZigbeeData_To_Fifo(GarageA_To1, 8);
					
					Car_L45(90, 400);
					Car_Left(wheel_Speed);
					Car_Track(Go_Speed);            // 主车循迹
          Car_Go(Go_Speed, Go_Temp);      // 主车前进					
					Car_R45(90, 400);
					//烽火台
          delay_ms(500);
          Infrared_Send(Alarm_Open, 6);   // 烽火台 -> 开启
          delay_ms(200);
					
					Car_L45(90, 400);
					Car_Left(wheel_Speed);
					Car_Track(Go_Speed);            // 主车循迹
          Car_Go(Go_Speed, Go_Temp);      // 主车前进					
					Car_Right(wheel_Speed);
					//智能路灯
          //Light_Inf(3);                   // 自动调节光照强度函数
					Infrared_Send(Light_plus2,4);
					delay_ms(500);
					
					Car_Left(wheel_Speed);
					Car_Track(Go_Speed);            // 主车循迹
          Car_Go(Go_Speed, Go_Temp);      // 主车前进					
					Car_Right(wheel_Speed);
					//障碍物
					Ultrasonic_Ranging();           // 采集超声波测距距离
          dis_size = dis;
          LED_Dis_Zigbee(dis_size);       // LED显示标志物发送测距信息
					delay_ms(100);
					LED_Dis_Zigbee(dis_size);       // LED显示标志物发送测距信息
					LED_Date_Zigbee(0x54,0x52,0x33,0x01);//显示学生学号
					
					Car_Left(wheel_Speed);
					Car_Left(wheel_Speed);
					Car_Track(Go_Speed);            // 主车循迹
          Car_Go(Go_Speed, Go_Temp);      // 主车前进
					Car_Track(Go_Speed);            // 主车循迹
          Car_Go(Go_Speed, Go_Temp);      // 主车前进
					//Car_Left(wheel_Speed);
					Car_L45(90, 400);
					//立体显示
					Rotate_show_Inf("545233",'B','2');
					
					//Car_Left(wheel_Speed);
					Car_L45(90, 400);
					
					Garage_Cont_Zigbee('A',1);		// 立体车库A 到达第1层
						while(Stop_Flag!=0x09)
						{
							//请求返回
							Send_ZigbeeData_To_Fifo(GarageA_Get_Floor,8);
							delay_ms(100);
							Can_ZigBeeRx_Check();
						}
						Car_Back(Go_Speed, 900);
						Car_Track(Go_Speed);
						Car_Back(Go_Speed, 1200);
						//delay_ms(100);
						//Car_Track(Go_Speed); 				//循迹
					
							//Car_Back(Go_Speed, 1500);		// 后退
						 	//Car_Track(30);	
							//Car_Back(Go_Speed, 2500);		// 后退
						 
							//Can_ZigBeeRx_Check();						
						//Car_Back(Go_Speed, 1900);	
						//Car_Back(Go_Speed, 320);

						Garage_Cont_Zigbee('A',4);		// 立体车库A 到达第3层
						//Send_ZigbeeData_To_Fifo(timefinish,8);
						TFT_Test_Zigbee('B',0x30,0x00,0x00,0x00);
						//Send_ZigbeeData_To_Fifo(timefinish,8);
						
					
					//TFT_Test_Zigbee('A',0x01 ,0x00,0x00,0x00);
					
					/*Send_ZigbeeData_To_Fifo(SMG_TimClear, 8);
					
					Send_ZigbeeData_To_Fifo(SEG_TimOpen, 8);    // LED显示标志物计时模式 -> 开启
          delay_ms(200);
          Send_ZigbeeData_To_Fifo(SEG_TimOpen, 8);    // LED显示标志物计时模式 -> 开启
          delay_ms(300);
					
					Car_Track(Go_Speed);            // 主车循迹
          Car_Go(Go_Speed, Go_Temp);      // 主车前进
					Car_Left(wheel_Speed);         // 主车左转 
					
					//Gate_Show_Zigbee("331723");//显示车牌
					delay_ms(500);
					
          Light_Inf(3);                   // 自动调节光照强度函数
					delay_ms(500);
					SYN_TTS("路灯已调[=tiao2]至3档[=dang3]");
					Car_Right(wheel_Speed);

					Car_Track(Go_Speed);            // 主车循迹
          Car_Go(Go_Speed, Go_Temp);      // 主车前进
					
					
					Car_Left(wheel_Speed);         // 主车左转
					
					TFT_Test_Zigbee('A',0x20,0x31,0x31,0x34);
					delay_ms(500);
					TFT_Test_Zigbee('A',0x21,0x35,0x31,0x34);  //TFT
					delay_ms(500);
					
					//Gate_Show_Zigbee("331723");//显示车牌
					
					Car_Right(wheel_Speed);         // 主车右转
					Car_Right(wheel_Speed);         // 主车右转
					
					
					Car_Track(Go_Speed);            // 主车循迹
          Car_Go(Go_Speed, Go_Temp);      // 主车前进
						
					Send_ZigbeeData_To_Fifo(Gate_Close, 8);//关闭闸门
					delay_ms(200); 
					
					Car_L45(90, 440);               // 旋转
          delay_ms(500);
  			  Infrared_Send(Rotate_2,6);		// 立体显示"前方施工，请绕行"
			    //Rotate_Dis_Inf(dis_size);		// 向立体显示物发送测距信息
//        Rotate_show_Inf("A123B4", 'F', '2');	// 向立体显示物发送车牌信息
          delay_ms(100);
          Car_R45(90, 440);               // 旋转
			    delay_ms(100);
					
					Car_Track(Go_Speed);            // 主车循迹
          Car_Go(Go_Speed, Go_Temp);      // 主车前进
					Car_Left(wheel_Speed);         // 主车左转
					Car_Track(Go_Speed);            // 主车循迹
          Car_Go(Go_Speed, Go_Temp);      // 主车前进
					Car_Right(wheel_Speed);         // 主车右转
					

					number = 5;         // 语音识别次数
          do 
					{
               SYN7318_Flag = SYN7318_Extern();
          }
          while ((!SYN7318_Flag) && (--number));
						
					int number=5;
					do
					{
					//YY_Comm_Zigbee(0x10,0x06);//标志物播放语音原地掉头SYN_Sesult[1]
					SYN7318_Extern();
					if(SYN7318_Flag)
						break;
					}while (--number);
					
					Car_Right(wheel_Speed);         // 主车右转
					Car_Track(Go_Speed);            // 主车循迹
          Car_Go(Go_Speed, Go_Temp);      // 主车前进
					delay_ms(100);
					Car_Track(Go_Speed);            // 主车循迹
          Car_Go(Go_Speed, Go_Temp);      // 主车前进
					Car_Left(wheel_Speed);         // 主车左转
					
					Ultrasonic_Ranging();           // 采集超声波测距距离
          dis_size = dis;
          LED_Dis_Zigbee(dis_size);       // LED显示标志物发送测距信息
					delay_ms(100);
					LED_Dis_Zigbee(dis_size);       // LED显示标志物发送测距信息
					//TFT_Dis_Zigbee('B',dis_size);	// TFT显示距离信息
					//TFT_Dis_Zigbee('B',dis_size);	// TFT显示距离信息
            
					delay_ms(200);
					
					Car_Right(wheel_Speed);         // 主车右转
					Car_Right(wheel_Speed);         // 主车右转
					Car_Track(Go_Speed);            // 主车循迹
          Car_Go(Go_Speed, Go_Temp);      // 主车前进
					
					Car_L45(90, 400);               // 旋转
          delay_ms(500);
          Infrared_Send(Alarm_Open, 6);   // 烽火台 -> 开启
          delay_ms(200);
					Car_R45(90, 400);               // 旋转
					
					Car_Right(wheel_Speed);         // 主车右转
					Car_Track(Go_Speed);            // 主车循迹
					Car_Go(Go_Speed, Go_Temp);      // 主车前进
					
					Send_ZigbeeData_To_Fifo(Gate_Open, 8); //开启闸门
					delay_ms(200);
					while(1)
					{
						Send_ZigbeeData_To_Fifo(Gate_GetStatus, 8);//查询状态
						delay_ms(20);
						Can_ZigBeeRx_Check(); 
						if(strcmp(Stop_Flag,0x05)==1)
								Send_UpMotor(0, 0);
						else if (strcmp(Stop_Flag,0x05)==0)
							break;
					}
					
					Car_Track(Go_Speed);            // 主车循迹
					Car_Go(Go_Speed, Go_Temp);      // 主车前进					
					Car_Left(wheel_Speed);         // 主车左转
					
					
					
						Garage_Cont_Zigbee('A',1);		// 立体车库A 到达第1层
						//delay_ms(100);
						//不断检测车库的位置
					
					Car_Track(Go_Speed);            // 主车循迹
					Car_Go(Go_Speed, Go_Temp);      // 主车前进					
					Car_Left(wheel_Speed);         // 主车左转
					Car_Left(wheel_Speed);         // 主车左转
					
						while(Stop_Flag!=0x09)
						{
							//请求返回
							Send_ZigbeeData_To_Fifo(GarageA_Get_Floor,8);
							delay_ms(100);
							Can_ZigBeeRx_Check();
						}
						
						//delay_ms(100);
						//Car_Track(Go_Speed); 				//循迹
					
						//Car_Back(Go_Speed, 1500);		// 后退
						Car_Track(30);	
						Car_Back(Go_Speed, 1500);		// 后退
						Car_Back(Go_Speed, 1200);		// 后退
						
						 
							Can_ZigBeeRx_Check();						
						//Car_Back(Go_Speed, 1900);	
						//Car_Back(Go_Speed, 320);

						Garage_Cont_Zigbee('A',3);		// 立体车库A 到达第3层
					
						*/
					 /*Car_Track(Go_Speed);            // 主车循迹
           Car_Go(Go_Speed, Go_Temp);      // 主车前进
					delay_ms(100); 
					Car_Left(wheel_Speed);         // 主车左转
					 //Send_UpMotor(0, 0);
					SYN7318_Put_String(mydef,69);//初始化语音
					  delay_ms(100);						
					Send_ZigbeeData_To_Fifo(SMG_TimClose, 8);//关闭计时
						delay_ms(100);
					
            Ultrasonic_Ranging();           // 采集超声波测距距离
            dis_size = dis;
            LED_Dis_Zigbee(dis_size);       // LED显示标志物发送测距信息
						delay_ms(100);
						LED_Dis_Zigbee(dis_size);       // LED显示标志物发送测距信息
						//TFT_Dis_Zigbee('B',dis_size);	// TFT显示距离信息
						//TFT_Dis_Zigbee('B',dis_size);	// TFT显示距离信息
            
						delay_ms(200);
						
					  /*Ultrasonic_Ranging();           // 采集超声波测距距离
            dis_size = dis;
            //LED_Dis_Zigbee(dis_size);       // LED显示标志物发送测距信息
				  	TFT_Dis_Zigbee('B',dis_size);	// TFT显示距离信息
					delay_ms(100);
					TFT_Dis_Zigbee('B',dis_size);	// TFT显示距离信*/

						
						//掉头
						/*Car_Left(wheel_Speed); 
						delay_ms(200);
						Car_Left(wheel_Speed); 
						delay_ms(200);
						// 开到B4
						Car_Track(Go_Speed);
						Car_Go(Go_Speed,Go_Temp);
						Car_Left(wheel_Speed);
						delay_ms(200);*/
						
						
						/*int num=3;
						do
						{
						//YY_Comm_Zigbee(0x10,0x06);//标志物播放语音原地掉头SYN_Sesult[1]
						SYN7318_Extern();
						if(SYN7318_Flag)
							break;
						}while (--num);*/
						
						
						//Garage_Cont_Zigbee('A',1);		// 立体车库A 到达第1层
						//delay_ms(100);
						//不断检测车库的位置
					/*	while(Stop_Flag!=0x09)
						{
							//请求返回
							Send_ZigbeeData_To_Fifo(GarageA_Get_Floor,8);
							delay_ms(100);
							Can_ZigBeeRx_Check();
						}*/
						
						//delay_ms(100);
						//Car_Track(Go_Speed); 				//循迹
					
							//Car_Back(Go_Speed, 1500);		// 后退
						 	//Car_Track(30);	
							//Car_Back(Go_Speed, 2500);		// 后退
						 
							//Can_ZigBeeRx_Check();						
						//Car_Back(Go_Speed, 1900);	
						//Car_Back(Go_Speed, 320);

						//Garage_Cont_Zigbee('A',3);		// 立体车库A 到达第3层
						
					}
				}
	
    if(S2 == 0)
    {
        delay_ms(10);
        if(S2 == 0)
        {
					LED2 = !LED2;
					//Send_ZigbeeData_To_Fifo(timestart,8);
					TFT_Test_Zigbee('A',0x30,0x01,0x00,0x00);
					delay_ms(100);
					TFT_Test_Zigbee('A',0x30,0x01,0x00,0x00);
					delay_ms(100);
					Send_ZigbeeData_To_Fifo(timestart,8);
			//Send_ZigbeeData_To_Fifo(AGV_Thread1,8);		// 移动机器人全自动调用
			//Send_Android(0xA4,0x00);		// 调用交通灯识别
					//SYN7318_Put_String(mydi,13);
					SYN7318_Put_String(mydef,69);//初始化语音
					//while(!S2);
        }					
            //LED1 = !LED1;
            //while(!S1);

            //make = 0x01;
    }

				
    if(S3 == 0)
    {
        delay_ms(10);
        if(S3 == 0)
        {
            LED3 = !LED3;
			
			TFT_Test_Zigbee('A',0x10,0x02,0x00,0x00);    // TFT显示器图片向下翻页
			
			RC522(40, RFID_Write_Read);  // RFID控制 -> 写卡
            while(!S3);
        }
    }
    if(S4 == 0)
    {
        delay_ms(10);
        if(S4 == 0)
        {
			number = 3;         // 语音识别次数
            do
            {
               SYN7318_Flag = SYN7318_Extern();//语音识别
							//SYN7318_Extern();
							//SYN7318_Test();
						}while ((!SYN7318_Flag)&& (--number));
						//((!SYN7318_Flag)) && 
            
			while(!S4);
        }
    }
}

int main(void)
{
    uint16_t Light_Value = 0;               // 光强度值
    uint16_t CodedDisk_Value = 0;           // 码盘值
    uint16_t Nav_Value = 0;                 // 角度值
	
    Hardware_Init();                        // 硬件初始化
	
    LED_twinkle_times = gt_get() + 50;
    Power_check_times = gt_get() + 200;
    WIFI_Upload_data_times = gt_get() + 200;
    RFID_Init_Check_times = gt_get() + 200;
    Principal_Tab[0] = 0x55;                // 主车数据上传指令包头
    Principal_Tab[1] = 0xAA;
    Follower_Tab[0] = 0x55;                 // 智能运输车数据上传指令包头
    Follower_Tab[1] = 0x02;
	Send_InfoData_To_Fifo("WEN\n", 5);
    Send_UpMotor(0, 0);
    while(1)
    {
        //Car_Thread();                                   // 全自动控制
        KEY_Check();                                    // 按键检测
        //Can_WifiRx_Check();                             // Wifi交互数据处理
        //Can_ZigBeeRx_Check();                           // Zigbee交互数据处理
  
    }
}

/* 硬件初始化函数 */
void Hardware_Init(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);     // 中断分组
    delay_init(168);                                    // 延时初始化
    Tba_Init();                                         // 任务板初始化
    Infrared_Init();                                    // 红外初始化
    Cba_Init();                                         // 核心板初始化
    Ultrasonic_Init();                                  // 超声波初始化
    Hard_Can_Init();                                    // CAN总线初始化
    BH1750_Configure();                                 // BH1750初始化配置
    SYN7318_Init();                                     // 语音识别初始化
    Electricity_Init();                                 // 电量检测初始化
    UartA72_Init();                                     // A72硬件串口通讯初始化
    Can_check_Init(7, 83);                              // CAN总线定时器初始化
    roadway_check_TimInit(999, 167);                   	// 路况检测
    Timer_Init(999, 167);                               // 串行数据通讯时间帧
    Readcard_daivce_Init();                         	// RFID初始化
}


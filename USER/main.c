/**
 * 2019���ۺ�����
 * �޸�ʱ�䣺2019.05.12
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
//��ͼ
uint8_t b_tu[6]={0xFF,0x12,0x06,0x00,0x00,0x00};
//�Զ���
uint8_t mydi[13]={0xFD,0x00,0x0A,0x53,0x01,0xD4,0xAD,0xC9,0xF1,0xC6,0xF4,0xB6,0xAF};
//����
uint8_t mydef[69]={0xFD,0x00,0x42,0x1F,0x01,0x33,0xC6,0xEB,0xCD,0xB7,0xB2,0xA2,0xBD,0xF8,0x20,0x31,0x31,0x30,0x7C,0xC3,0xC0,0xBA,0xC3,0xC9,0xFA,0xBB,0xEE,0x20,0x31,0x31,0x31,0x7C,0xD0,0xE3,0xC0,0xF6,0xC9,0xBD,0xBA,0xD3,0x20,0x31,0x31,0x32,0x7C,0xD7,0xB7,0xD6,0xF0,0xC3,0xCE,0xCF,0xEB,0x20,0x31,0x31,0x33,0x7C,0x7C,0x7C,0x7C,0x7C,0x7C,0x7C,0x7C,0x7C,0x7C,0x7C,0x7C};
//��������
void SYN7318_Put_String(uint8_t* Pst,uint8_t Length);
/***************************** �Զ������ *****************************/
static uint8_t Go_Speed  = 50;              // ȫ��ǰ���ٶ�ֵ
static uint8_t wheel_Speed = 95;            // ȫ��ת���ٶ�ֵ
static uint16_t Go_Temp = 450;              // ȫ��ת���ٶ�ֵ

static uint32_t Power_check_times;          // �����������
static uint32_t LED_twinkle_times;          // LED��˸����
static uint32_t WIFI_Upload_data_times;     // ͨ��Wifi�ϴ���������
static uint32_t RFID_Init_Check_times;      // RFID��ʼ�����ʱ������

uint8_t make = 0;                   // ȫ�Զ���ʻ��־λ
uint8_t RFID_Flag = 0;          	// RFID����־λ
uint8_t SYN7318_Flag = 0;           // SYN7318����ʶ������ID���
uint8_t number = 0;                 // ����ֵ
uint8_t RFID_addr = 0;				// RFID��Ч���ݿ��ַ
uint8_t Terrain_Flag = 0;			// ���μ���־λ
uint16_t dis_size = 0;              // ���������ֵ����
uint16_t tim_i,tim_j;

uint8_t AGV_GO_sp = 50;				// С��ǰ���ٶ�
uint8_t AGV_wheel_sp = 90;			// С��ת���ٶ�
uint16_t AGV_GO_mp = 420;			// С��ǰ������

uint8_t SYN_Sesult[8] = {0xAF,0x06,0x00,0x02,0x00,0x00,0x01,0xBF};	// ����ʶ�����ϴ�Э��

static uint8_t timestart[8] = {0x55,0x08,0x30,0x01,0x00,0x00,0x31,0xBB};
static uint8_t timefinish[8] = {0x55,0x08,0x30,0x00,0x00,0x00,0x30,0xBB};


static void Car_Thread(void);               // ȫ�Զ�����
static void KEY_Check(void);                // ������⺯��
static void Hardware_Init(void);            // Ӳ����ʼ������
/**********************************************************************/

#if 0

/* ȫ�Զ����к��� */
void Car_Thread(void)
{
    switch(make)
    {
        case 0x01:
        {
            Send_ZigbeeData_To_Fifo(SEG_TimOpen, 8);    // LED��ʾ��־���ʱģʽ -> ����
            delay_ms(200);
            Send_ZigbeeData_To_Fifo(SEG_TimOpen, 8);    // LED��ʾ��־���ʱģʽ -> ����
            delay_ms(300);
			
			TFT_Test_Zigbee('A',0x40,0x20,0x19,0x05);    // TFT��ʾ����
			delay_ms(200);
			TFT_Test_Zigbee('A',0x40,0x20,0x19,0x05);    // TFT��ʾ����
			delay_ms(100);
			
			Send_Android(0xA1, 0x01);		// ��������ͷԤ��λ1
			
            Car_Track(Go_Speed);            // ����ѭ��
            Car_Go(Go_Speed, Go_Temp);      // ����ǰ��
			Car_Right(wheel_Speed);         // ������ת
			Car_Time_Track(Go_Speed, 500);	// ����ʱ��ѭ��
			
//---------------------- ���ܽ�ͨ�Ʊ�־�� ----------------------
            delay_ms(500);
            Send_ZigbeeData_To_Fifo(TrafficA_Open, 8);   // ���ܽ�ͨ�ƽ���ʶ��ģʽ
            delay_ms(500); delay_ms(500); delay_ms(500);
            Send_ZigbeeData_To_Fifo(TrafficA_Yellow, 8);    // ���ܽ�ͨ�� -> ��ɫ
            delay_ms(300);
			
            Car_Track(Go_Speed);            // ����ѭ��
            Car_Go(Go_Speed, Go_Temp);		// ����ǰ��
			Car_Track(Go_Speed);            // ����ѭ��

//------------------- ��̬��־�� --------------------------
            delay_ms(500);
            Ultrasonic_Ranging();           // �ɼ�������������
            dis_size = dis;
            LED_Dis_Zigbee(dis_size);       // LED��ʾ��־�﷢�Ͳ����Ϣ
			LED_Dis_Zigbee(dis_size);       // LED��ʾ��־�﷢�Ͳ����Ϣ
			TFT_Dis_Zigbee('B',dis_size);	// TFT��ʾ������Ϣ
			TFT_Dis_Zigbee('B',dis_size);	// TFT��ʾ������Ϣ
			
			Car_Go(Go_Speed, Go_Temp+10);   // ����ǰ��
			Car_Left(wheel_Speed);          // ������ת
			
//------------------- ������ʾ -------------------------
            Car_L45(90, 440);               // ��ת
            delay_ms(500);
//			Infrared_Send(Rotate_2,6);		// ������ʾ"ǰ��ʩ����������"
			Rotate_Dis_Inf(dis_size);		// ��������ʾ�﷢�Ͳ����Ϣ
//            Rotate_show_Inf("A123B4", 'F', '2');	// ��������ʾ�﷢�ͳ�����Ϣ
            delay_ms(100);
            Car_R45(90, 440);               // ��ת
			delay_ms(100);
			
			make = 0x02;
			break;
		}
		
		case 0x02:
		{
//------------------- ���μ�� ---------------------------
			Terrain_Flag = 1;
            Car_Track(Go_Speed);            // ����ѭ��
			Terrain_Flag = 0;				// ������μ���־λ
			
			Car_Go(Go_Speed, Go_Temp+10);   // ����ǰ��
			Car_Right(wheel_Speed);         // ������ת
			
//------------------- ����·�� ---------------------------
            delay_ms(500);
            Light_Inf(3);                   // �Զ����ڹ���ǿ�Ⱥ���
			delay_ms(500);
			SYN_TTS("·���ѵ�[=tiao2]��3��[=dang3]");
			
			Car_Left(wheel_Speed);          // ������ת
			Car_Left(wheel_Speed);          // ������ת
			
//------------------- RFID ---------------------------
			RFID_addr = 40;					// RFID��Ч���ݿ��ַ
			RFID_Flag = 10;					// RFID����־λ/�н��ٶȽ��Ͳ���
			CarThread_Track(Go_Speed - RFID_Flag); // ����ѭ��
			make = 0x03;
			break;
		}
			
		case 0x03:
		{
			if (Stop_Flag == 0x01)
			{
				make = 0x04;
			}
			if ((RFID_Flag != 0) && (PcdRequest(PICC_REQALL,CT) == MI_OK))		// RFIDѰ��
			{
				Roadway_Flag_clean();	// �����־λ״̬
				Send_UpMotor(0,0);		// ͣ��
				delay_ms(100);
				if (Read_RFID(RFID_addr) == 0x02)		// RFID ���� -> READ_RFID[16]
				{
					MP_SPK = 1;
					delay_ms(500);
					MP_SPK = 0;
					SYN_TTS("�����ɹ�");
					RFID_Flag = 0;				// ���RFID��־λ
					CarThread_Track(Go_Speed);	// ����ѭ��
				}
				else
				{
					CarThread_Track(Go_Speed - RFID_Flag); // ����ѭ��
				}
			}
			break;
		}
			
		case 0x04:
		{
			RFID_Flag = 0;					// ���RFID��־λ
			Car_Go(Go_Speed, Go_Temp);   	// ����ǰ��
			Car_Right(wheel_Speed);         // ������ת
			Car_Time_Track(Go_Speed, 500);	// ����ʱ��ѭ��
			
//-------------------- ����TFT��ʾ�� --------------------
            delay_ms(500);
			delay_ms(500);
//          TFT_Test_Zigbee('A',0x10,0x01,0x00,0x00);   // TFT��ʾ��ͼƬ���Ϸ�ҳ
			TFT_Test_Zigbee('A',0x10,0x02,0x00,0x00);   // TFT��ʾ��ͼƬ���·�ҳ
			delay_ms(500);
			TFT_Test_Zigbee('A',0x10,0x02,0x00,0x00);   // TFT��ʾ��ͼƬ���·�ҳ
			
			delay_ms(600); delay_ms(600); delay_ms(600); delay_ms(600);
			TFT_Test_Zigbee('A',0x10,0x02,0x00,0x00);   // TFT��ʾ��ͼƬ���·�ҳ
			delay_ms(500);
			TFT_Test_Zigbee('A',0x10,0x02,0x00,0x00);   // TFT��ʾ��ͼƬ���·�ҳ
			
			delay_ms(600); delay_ms(600); delay_ms(600); delay_ms(600); 
			TFT_Test_Zigbee('A',0x40,0x12,0x34,0x56);   // TFT��ʾ����ʾͼ����Ϣ
			delay_ms(200);
			TFT_Test_Zigbee('A',0x40,0x12,0x34,0x56);   // TFT��ʾ����ʾͼ����Ϣ
			delay_ms(100);
//			TFT_Show_Zigbee('A',"A88888");              // TFT��ʾ����ʾͼ����Ϣ
			delay_ms(500);
			
            Car_Back(Go_Speed, 320);		// ����
			Car_Left(wheel_Speed);          // ������ת
#if 0
			Car_Time_Track(Go_Speed, 500);	// ʱ��ѭ��
			
//---------------------- ���ܽ�ͨ�Ʊ�־�� ----------------------
            delay_ms(100);
            Send_ZigbeeData_To_Fifo(TrafficB_Open, 8);   // ���ܽ�ͨ�ƽ���ʶ��ģʽ
            delay_ms(500);
            delay_ms(500);
            Send_ZigbeeData_To_Fifo(TrafficB_Yellow, 8);	// ���ܽ�ͨ�� -> ��ɫ
            delay_ms(100);
#endif 
            Car_Track(Go_Speed);            // ����ѭ��
			
//---------------------- ����������־�� ----------------------
			number = 3;         // ����ʶ�����
            do 
						{
                SYN7318_Flag = SYN7318_Extern();
            }
            while ((!SYN7318_Flag) && (--number));
						
						
            SYN_Sesult[2] = SYN7318_Flag;
            Send_ZigbeeData_To_Fifo(SYN_Sesult, 8);    // �ϴ�����������
            delay_ms(100);
			
//------------------- �ƶ�������ȫ�Զ����� --------------------
			SYN_TTS("С��� ����");
			Send_ZigbeeData_To_Fifo(AGV_Thread1,8);		// �ƶ�������ȫ�Զ�����
			delay_ms(200);
			Send_ZigbeeData_To_Fifo(AGV_Thread1,8);		// �ƶ�������ȫ�Զ�����
			delay_ms(200);
			Send_ZigbeeData_To_Fifo(AGV_Thread1,8);		// �ƶ�������ȫ�Զ�����
			delay_ms(100);
			
//-------------------- ���峵�⸴λ ----------------------------
			Send_ZigbeeData_To_Fifo(GarageA_To1, 8);	// ���⵽���һ��
			delay_ms(200);
			Send_ZigbeeData_To_Fifo(GarageA_To1, 8);	// ���⵽���һ��
			delay_ms(200);
			
			Car_Go(Go_Speed, Go_Temp+10);   // ����ǰ��
			Car_Left(wheel_Speed);          // ������ת
			
//-------------------- ��բϵͳ --------------------
//			Gate_Show_Zigbee("A88888");		// ��բ��ʾ����
			Send_ZigbeeData_To_Fifo(Gate_Open, 8);       // ��բ -> ����
			//0x55,0x03,0x01,0x01,0x00,0x00,0x02 ,0xbb
			
			delay_ms(200);
			Send_ZigbeeData_To_Fifo(Gate_Open, 8);       // ��բ -> ����
			delay_ms(500);
			delay_ms(500);
			
			Car_Track(Go_Speed);            // ����ѭ��
			Car_Go(Go_Speed, Go_Temp);   	// ����ǰ��
			Car_Left(wheel_Speed);          // ������ת
			Car_Time_Track(Go_Speed, 100);	// ʱ��ѭ��
			
//-------------------- ETC���� ------------------------
			ETC_Get_Zigbee();				// ETCϵͳ���
			
			TFT_Show_Zigbee('A',"A88888");    // TFT��ʾ����ʾͼ����Ϣ
			delay_ms(300);
			TFT_Show_Zigbee('A',"A88888");    // TFT��ʾ����ʾͼ����Ϣ
			delay_ms(100);
			
			Car_Track(Go_Speed);            // ����ѭ��
			Car_Go(Go_Speed, Go_Temp+10);   // ����ǰ��
			
//------------------- ���̨ ---------------------------
			Car_L45(90, 400);               // ��ת
            delay_ms(500);
            Infrared_Send(Alarm_Open, 6);   // ���̨ -> ����
            delay_ms(200);
			Car_R45(90, 400);               // ��ת
			
			Car_Right(wheel_Speed);         // ������ת
			Car_Track(Go_Speed);            // ����ѭ��
			Car_Go(Go_Speed, Go_Temp);      // ����ǰ��
			Car_Left(wheel_Speed);          // ������ת
			Car_Track(Go_Speed);            // ����ѭ��
			
//------------------- ��̬��־�� --------------------------
            delay_ms(500);
            Ultrasonic_Ranging();           // �ɼ�������������
            dis_size = dis;
            LED_Dis_Zigbee(dis_size);       // LED��ʾ��־�﷢�Ͳ����Ϣ
			LED_Dis_Zigbee(dis_size);       // LED��ʾ��־�﷢�Ͳ����Ϣ
			TFT_Dis_Zigbee('B',dis_size);	// TFT��ʾ������Ϣ
			TFT_Dis_Zigbee('B',dis_size);	// TFT��ʾ������Ϣ
			
			Car_Go(Go_Speed, Go_Temp-10);   	// ����ǰ��
			Car_Left(wheel_Speed);          // ������ת
			Car_Time_Track(Go_Speed, 800);	// ʱ��ѭ��
			
//------------------- ���峵��A ---------------------------
			Garage_Cont_Zigbee('A',1);		// ���峵��A �����1��
			
			Car_Back(Go_Speed, 1900);		// ����
			
			Garage_Cont_Zigbee('A',3);		// ���峵��A �����3��
			
            Send_ZigbeeData_To_Fifo(Charge_Open, 8);  // ���߳�� -> ����
            delay_ms(200);
            Send_ZigbeeData_To_Fifo(Charge_Open, 8);  // ���߳�� -> ����
            delay_ms(100);
			
//			AGV_Data_Open();				// AGV���������ϴ�
//			AGV_GetThread(0xA1);			// AGVȫ�Զ���ɱ�־��ȡ
//			AGV_Data_Stop();				// AGV�ر������ϴ�
			
            delay_ms(100);
            Send_ZigbeeData_To_Fifo(SMG_TimClose, 8);  // ����ܼ�ʱ -> �ر�
            delay_ms(300);
            Send_ZigbeeData_To_Fifo(SMG_TimClose, 8);  // ����ܼ�ʱ -> �ر�
            delay_ms(500);

            Set_tba_Beep(SET);      // ���������� -> ����
            delay_ms(500);
            delay_ms(500);
            delay_ms(500);
            Set_tba_Beep(RESET);    // ���������� -> �ر�
			
			make = 0x05;
			break;
		}
			
		case 0x05:
		{
//------------------- ���峵��A ---------------------------
			SYN_TTS("3�������ȫ�Զ�");
			delay_ms(100);
			
//------------------- ɳ�̸�λ ---------------------------		
			TFT_Test_Zigbee('A',0x10,0x02,0x00,0x00);    // TFT��ʾ��ͼƬ���·�ҳ
			delay_ms(500);
			TFT_Test_Zigbee('A',0x10,0x02,0x00,0x00);    // TFT��ʾ��ͼƬ���·�ҳ
			delay_ms(100);
			TFT_Test_Zigbee('B',0x10,0x02,0x00,0x00);    // TFT��ʾ��ͼƬ���·�ҳ
			delay_ms(500);
			TFT_Test_Zigbee('B',0x10,0x02,0x00,0x00);    // TFT��ʾ��ͼƬ���·�ҳ
			delay_ms(100);
			
			Send_ZigbeeData_To_Fifo(SEG_TimOpen, 8);    // LED��ʾ��־���ʱģʽ -> ����
            delay_ms(300);
            Send_ZigbeeData_To_Fifo(SEG_TimOpen, 8);    // LED��ʾ��־���ʱģʽ -> ����
            delay_ms(300);
			
			Garage_Cont_Zigbee('A',1);		// ���峵��A �����1��
			delay_ms(100);
			
			Car_Go(Go_Speed, 500);      	// ����ǰ��
			Car_Track(Go_Speed);            // ����ѭ��
            Car_Go(Go_Speed, Go_Temp);      // ����ǰ��
			
//------------------- ������ʾ -------------------------
            Car_L45(90, 450);               // ��ת
            delay_ms(500);
			Infrared_Send(Rotate_2,6);		// ������ʾ"ǰ��ʩ����������"
            delay_ms(100);
            Car_R45(90, 450);               // ��ת
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

/* ������⺯�� */
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
		//̩

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
					
					Car_Track(Go_Speed);            // ����ѭ��
          Car_Go(Go_Speed, Go_Temp);      // ����ǰ��
					Car_Left(wheel_Speed);         // ������ת
					Car_Track(Go_Speed);            // ����ѭ��
          Car_Go(Go_Speed, Go_Temp);      // ����ǰ��
					Car_Right(wheel_Speed);
					//��բ
					Send_ZigbeeData_To_Fifo(Gate_Open, 8); //����բ��
					delay_ms(150);
					while(1)
					{
						Send_ZigbeeData_To_Fifo(Gate_GetStatus, 8);//��ѯ״̬
						delay_ms(20);
						Can_ZigBeeRx_Check(); 
						if(strcmp(Stop_Flag,0x05)==1)
								Send_UpMotor(0, 0);
						else if (strcmp(Stop_Flag,0x05)==0)
							break;
					}					
					Gate_Show_Zigbee("545233");//��ʾ����
					
					
					
					Car_Track(Go_Speed);            // ����ѭ��
          Car_Go(Go_Speed, Go_Temp);      // ����ǰ��	
					Send_ZigbeeData_To_Fifo(Gate_Close, 8);//�ر�բ��					
					Car_Right(wheel_Speed);
					
					
					Car_Track(Go_Speed);            // ����ѭ��
          Car_Go(Go_Speed, Go_Temp);      // ����ǰ��	
					Car_R45(90, 400);
					//��������
					
					number = 5;         // ����ʶ�����
          do 
					{
               SYN7318_Flag = SYN7318_Extern();
          }
          while ((!SYN7318_Flag) && (--number));
						
					/*int number=5;
					do
					{
					//YY_Comm_Zigbee(0x10,0x06);//��־�ﲥ������ԭ�ص�ͷSYN_Sesult[1]
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
					Car_Track(Go_Speed);            // ����ѭ��
          Car_Go(Go_Speed, Go_Temp);      // ����ǰ��					
					Car_R45(90, 400);
					//���̨
          delay_ms(500);
          Infrared_Send(Alarm_Open, 6);   // ���̨ -> ����
          delay_ms(200);
					
					Car_L45(90, 400);
					Car_Left(wheel_Speed);
					Car_Track(Go_Speed);            // ����ѭ��
          Car_Go(Go_Speed, Go_Temp);      // ����ǰ��					
					Car_Right(wheel_Speed);
					//����·��
          //Light_Inf(3);                   // �Զ����ڹ���ǿ�Ⱥ���
					Infrared_Send(Light_plus2,4);
					delay_ms(500);
					
					Car_Left(wheel_Speed);
					Car_Track(Go_Speed);            // ����ѭ��
          Car_Go(Go_Speed, Go_Temp);      // ����ǰ��					
					Car_Right(wheel_Speed);
					//�ϰ���
					Ultrasonic_Ranging();           // �ɼ�������������
          dis_size = dis;
          LED_Dis_Zigbee(dis_size);       // LED��ʾ��־�﷢�Ͳ����Ϣ
					delay_ms(100);
					LED_Dis_Zigbee(dis_size);       // LED��ʾ��־�﷢�Ͳ����Ϣ
					LED_Date_Zigbee(0x54,0x52,0x33,0x01);//��ʾѧ��ѧ��
					
					Car_Left(wheel_Speed);
					Car_Left(wheel_Speed);
					Car_Track(Go_Speed);            // ����ѭ��
          Car_Go(Go_Speed, Go_Temp);      // ����ǰ��
					Car_Track(Go_Speed);            // ����ѭ��
          Car_Go(Go_Speed, Go_Temp);      // ����ǰ��
					//Car_Left(wheel_Speed);
					Car_L45(90, 400);
					//������ʾ
					Rotate_show_Inf("545233",'B','2');
					
					//Car_Left(wheel_Speed);
					Car_L45(90, 400);
					
					Garage_Cont_Zigbee('A',1);		// ���峵��A �����1��
						while(Stop_Flag!=0x09)
						{
							//���󷵻�
							Send_ZigbeeData_To_Fifo(GarageA_Get_Floor,8);
							delay_ms(100);
							Can_ZigBeeRx_Check();
						}
						Car_Back(Go_Speed, 900);
						Car_Track(Go_Speed);
						Car_Back(Go_Speed, 1200);
						//delay_ms(100);
						//Car_Track(Go_Speed); 				//ѭ��
					
							//Car_Back(Go_Speed, 1500);		// ����
						 	//Car_Track(30);	
							//Car_Back(Go_Speed, 2500);		// ����
						 
							//Can_ZigBeeRx_Check();						
						//Car_Back(Go_Speed, 1900);	
						//Car_Back(Go_Speed, 320);

						Garage_Cont_Zigbee('A',4);		// ���峵��A �����3��
						//Send_ZigbeeData_To_Fifo(timefinish,8);
						TFT_Test_Zigbee('B',0x30,0x00,0x00,0x00);
						//Send_ZigbeeData_To_Fifo(timefinish,8);
						
					
					//TFT_Test_Zigbee('A',0x01 ,0x00,0x00,0x00);
					
					/*Send_ZigbeeData_To_Fifo(SMG_TimClear, 8);
					
					Send_ZigbeeData_To_Fifo(SEG_TimOpen, 8);    // LED��ʾ��־���ʱģʽ -> ����
          delay_ms(200);
          Send_ZigbeeData_To_Fifo(SEG_TimOpen, 8);    // LED��ʾ��־���ʱģʽ -> ����
          delay_ms(300);
					
					Car_Track(Go_Speed);            // ����ѭ��
          Car_Go(Go_Speed, Go_Temp);      // ����ǰ��
					Car_Left(wheel_Speed);         // ������ת 
					
					//Gate_Show_Zigbee("331723");//��ʾ����
					delay_ms(500);
					
          Light_Inf(3);                   // �Զ����ڹ���ǿ�Ⱥ���
					delay_ms(500);
					SYN_TTS("·���ѵ�[=tiao2]��3��[=dang3]");
					Car_Right(wheel_Speed);

					Car_Track(Go_Speed);            // ����ѭ��
          Car_Go(Go_Speed, Go_Temp);      // ����ǰ��
					
					
					Car_Left(wheel_Speed);         // ������ת
					
					TFT_Test_Zigbee('A',0x20,0x31,0x31,0x34);
					delay_ms(500);
					TFT_Test_Zigbee('A',0x21,0x35,0x31,0x34);  //TFT
					delay_ms(500);
					
					//Gate_Show_Zigbee("331723");//��ʾ����
					
					Car_Right(wheel_Speed);         // ������ת
					Car_Right(wheel_Speed);         // ������ת
					
					
					Car_Track(Go_Speed);            // ����ѭ��
          Car_Go(Go_Speed, Go_Temp);      // ����ǰ��
						
					Send_ZigbeeData_To_Fifo(Gate_Close, 8);//�ر�բ��
					delay_ms(200); 
					
					Car_L45(90, 440);               // ��ת
          delay_ms(500);
  			  Infrared_Send(Rotate_2,6);		// ������ʾ"ǰ��ʩ����������"
			    //Rotate_Dis_Inf(dis_size);		// ��������ʾ�﷢�Ͳ����Ϣ
//        Rotate_show_Inf("A123B4", 'F', '2');	// ��������ʾ�﷢�ͳ�����Ϣ
          delay_ms(100);
          Car_R45(90, 440);               // ��ת
			    delay_ms(100);
					
					Car_Track(Go_Speed);            // ����ѭ��
          Car_Go(Go_Speed, Go_Temp);      // ����ǰ��
					Car_Left(wheel_Speed);         // ������ת
					Car_Track(Go_Speed);            // ����ѭ��
          Car_Go(Go_Speed, Go_Temp);      // ����ǰ��
					Car_Right(wheel_Speed);         // ������ת
					

					number = 5;         // ����ʶ�����
          do 
					{
               SYN7318_Flag = SYN7318_Extern();
          }
          while ((!SYN7318_Flag) && (--number));
						
					int number=5;
					do
					{
					//YY_Comm_Zigbee(0x10,0x06);//��־�ﲥ������ԭ�ص�ͷSYN_Sesult[1]
					SYN7318_Extern();
					if(SYN7318_Flag)
						break;
					}while (--number);
					
					Car_Right(wheel_Speed);         // ������ת
					Car_Track(Go_Speed);            // ����ѭ��
          Car_Go(Go_Speed, Go_Temp);      // ����ǰ��
					delay_ms(100);
					Car_Track(Go_Speed);            // ����ѭ��
          Car_Go(Go_Speed, Go_Temp);      // ����ǰ��
					Car_Left(wheel_Speed);         // ������ת
					
					Ultrasonic_Ranging();           // �ɼ�������������
          dis_size = dis;
          LED_Dis_Zigbee(dis_size);       // LED��ʾ��־�﷢�Ͳ����Ϣ
					delay_ms(100);
					LED_Dis_Zigbee(dis_size);       // LED��ʾ��־�﷢�Ͳ����Ϣ
					//TFT_Dis_Zigbee('B',dis_size);	// TFT��ʾ������Ϣ
					//TFT_Dis_Zigbee('B',dis_size);	// TFT��ʾ������Ϣ
            
					delay_ms(200);
					
					Car_Right(wheel_Speed);         // ������ת
					Car_Right(wheel_Speed);         // ������ת
					Car_Track(Go_Speed);            // ����ѭ��
          Car_Go(Go_Speed, Go_Temp);      // ����ǰ��
					
					Car_L45(90, 400);               // ��ת
          delay_ms(500);
          Infrared_Send(Alarm_Open, 6);   // ���̨ -> ����
          delay_ms(200);
					Car_R45(90, 400);               // ��ת
					
					Car_Right(wheel_Speed);         // ������ת
					Car_Track(Go_Speed);            // ����ѭ��
					Car_Go(Go_Speed, Go_Temp);      // ����ǰ��
					
					Send_ZigbeeData_To_Fifo(Gate_Open, 8); //����բ��
					delay_ms(200);
					while(1)
					{
						Send_ZigbeeData_To_Fifo(Gate_GetStatus, 8);//��ѯ״̬
						delay_ms(20);
						Can_ZigBeeRx_Check(); 
						if(strcmp(Stop_Flag,0x05)==1)
								Send_UpMotor(0, 0);
						else if (strcmp(Stop_Flag,0x05)==0)
							break;
					}
					
					Car_Track(Go_Speed);            // ����ѭ��
					Car_Go(Go_Speed, Go_Temp);      // ����ǰ��					
					Car_Left(wheel_Speed);         // ������ת
					
					
					
						Garage_Cont_Zigbee('A',1);		// ���峵��A �����1��
						//delay_ms(100);
						//���ϼ�⳵���λ��
					
					Car_Track(Go_Speed);            // ����ѭ��
					Car_Go(Go_Speed, Go_Temp);      // ����ǰ��					
					Car_Left(wheel_Speed);         // ������ת
					Car_Left(wheel_Speed);         // ������ת
					
						while(Stop_Flag!=0x09)
						{
							//���󷵻�
							Send_ZigbeeData_To_Fifo(GarageA_Get_Floor,8);
							delay_ms(100);
							Can_ZigBeeRx_Check();
						}
						
						//delay_ms(100);
						//Car_Track(Go_Speed); 				//ѭ��
					
						//Car_Back(Go_Speed, 1500);		// ����
						Car_Track(30);	
						Car_Back(Go_Speed, 1500);		// ����
						Car_Back(Go_Speed, 1200);		// ����
						
						 
							Can_ZigBeeRx_Check();						
						//Car_Back(Go_Speed, 1900);	
						//Car_Back(Go_Speed, 320);

						Garage_Cont_Zigbee('A',3);		// ���峵��A �����3��
					
						*/
					 /*Car_Track(Go_Speed);            // ����ѭ��
           Car_Go(Go_Speed, Go_Temp);      // ����ǰ��
					delay_ms(100); 
					Car_Left(wheel_Speed);         // ������ת
					 //Send_UpMotor(0, 0);
					SYN7318_Put_String(mydef,69);//��ʼ������
					  delay_ms(100);						
					Send_ZigbeeData_To_Fifo(SMG_TimClose, 8);//�رռ�ʱ
						delay_ms(100);
					
            Ultrasonic_Ranging();           // �ɼ�������������
            dis_size = dis;
            LED_Dis_Zigbee(dis_size);       // LED��ʾ��־�﷢�Ͳ����Ϣ
						delay_ms(100);
						LED_Dis_Zigbee(dis_size);       // LED��ʾ��־�﷢�Ͳ����Ϣ
						//TFT_Dis_Zigbee('B',dis_size);	// TFT��ʾ������Ϣ
						//TFT_Dis_Zigbee('B',dis_size);	// TFT��ʾ������Ϣ
            
						delay_ms(200);
						
					  /*Ultrasonic_Ranging();           // �ɼ�������������
            dis_size = dis;
            //LED_Dis_Zigbee(dis_size);       // LED��ʾ��־�﷢�Ͳ����Ϣ
				  	TFT_Dis_Zigbee('B',dis_size);	// TFT��ʾ������Ϣ
					delay_ms(100);
					TFT_Dis_Zigbee('B',dis_size);	// TFT��ʾ������*/

						
						//��ͷ
						/*Car_Left(wheel_Speed); 
						delay_ms(200);
						Car_Left(wheel_Speed); 
						delay_ms(200);
						// ����B4
						Car_Track(Go_Speed);
						Car_Go(Go_Speed,Go_Temp);
						Car_Left(wheel_Speed);
						delay_ms(200);*/
						
						
						/*int num=3;
						do
						{
						//YY_Comm_Zigbee(0x10,0x06);//��־�ﲥ������ԭ�ص�ͷSYN_Sesult[1]
						SYN7318_Extern();
						if(SYN7318_Flag)
							break;
						}while (--num);*/
						
						
						//Garage_Cont_Zigbee('A',1);		// ���峵��A �����1��
						//delay_ms(100);
						//���ϼ�⳵���λ��
					/*	while(Stop_Flag!=0x09)
						{
							//���󷵻�
							Send_ZigbeeData_To_Fifo(GarageA_Get_Floor,8);
							delay_ms(100);
							Can_ZigBeeRx_Check();
						}*/
						
						//delay_ms(100);
						//Car_Track(Go_Speed); 				//ѭ��
					
							//Car_Back(Go_Speed, 1500);		// ����
						 	//Car_Track(30);	
							//Car_Back(Go_Speed, 2500);		// ����
						 
							//Can_ZigBeeRx_Check();						
						//Car_Back(Go_Speed, 1900);	
						//Car_Back(Go_Speed, 320);

						//Garage_Cont_Zigbee('A',3);		// ���峵��A �����3��
						
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
			//Send_ZigbeeData_To_Fifo(AGV_Thread1,8);		// �ƶ�������ȫ�Զ�����
			//Send_Android(0xA4,0x00);		// ���ý�ͨ��ʶ��
					//SYN7318_Put_String(mydi,13);
					SYN7318_Put_String(mydef,69);//��ʼ������
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
			
			TFT_Test_Zigbee('A',0x10,0x02,0x00,0x00);    // TFT��ʾ��ͼƬ���·�ҳ
			
			RC522(40, RFID_Write_Read);  // RFID���� -> д��
            while(!S3);
        }
    }
    if(S4 == 0)
    {
        delay_ms(10);
        if(S4 == 0)
        {
			number = 3;         // ����ʶ�����
            do
            {
               SYN7318_Flag = SYN7318_Extern();//����ʶ��
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
    uint16_t Light_Value = 0;               // ��ǿ��ֵ
    uint16_t CodedDisk_Value = 0;           // ����ֵ
    uint16_t Nav_Value = 0;                 // �Ƕ�ֵ
	
    Hardware_Init();                        // Ӳ����ʼ��
	
    LED_twinkle_times = gt_get() + 50;
    Power_check_times = gt_get() + 200;
    WIFI_Upload_data_times = gt_get() + 200;
    RFID_Init_Check_times = gt_get() + 200;
    Principal_Tab[0] = 0x55;                // ���������ϴ�ָ���ͷ
    Principal_Tab[1] = 0xAA;
    Follower_Tab[0] = 0x55;                 // �������䳵�����ϴ�ָ���ͷ
    Follower_Tab[1] = 0x02;
	Send_InfoData_To_Fifo("WEN\n", 5);
    Send_UpMotor(0, 0);
    while(1)
    {
        //Car_Thread();                                   // ȫ�Զ�����
        KEY_Check();                                    // �������
        //Can_WifiRx_Check();                             // Wifi�������ݴ���
        //Can_ZigBeeRx_Check();                           // Zigbee�������ݴ���
  
    }
}

/* Ӳ����ʼ������ */
void Hardware_Init(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);     // �жϷ���
    delay_init(168);                                    // ��ʱ��ʼ��
    Tba_Init();                                         // ������ʼ��
    Infrared_Init();                                    // �����ʼ��
    Cba_Init();                                         // ���İ��ʼ��
    Ultrasonic_Init();                                  // ��������ʼ��
    Hard_Can_Init();                                    // CAN���߳�ʼ��
    BH1750_Configure();                                 // BH1750��ʼ������
    SYN7318_Init();                                     // ����ʶ���ʼ��
    Electricity_Init();                                 // ��������ʼ��
    UartA72_Init();                                     // A72Ӳ������ͨѶ��ʼ��
    Can_check_Init(7, 83);                              // CAN���߶�ʱ����ʼ��
    roadway_check_TimInit(999, 167);                   	// ·�����
    Timer_Init(999, 167);                               // ��������ͨѶʱ��֡
    Readcard_daivce_Init();                         	// RFID��ʼ��
}



#ifndef __DATA_CHANNEL_H__
#define __DATA_CHANNEL_H__
#include "stm32f4xx.h"

#ifdef __DATA_CHANNEL_C__
#define GLOBAL
#else
#define GLOBAL extern
#endif

#define  ZIGB_RX_MAX    200
#define  WIFI_MAX_NUM   200

GLOBAL uint8_t Wifi_Rx_Buf[ WIFI_MAX_NUM ];
GLOBAL uint8_t Zigb_Rx_Buf[ ZIGB_RX_MAX ];


GLOBAL uint8_t Wifi_Rx_num ;
GLOBAL uint8_t Wifi_Rx_flag ;  //接收完成标志位
GLOBAL uint8_t Zigbee_Rx_num ;
GLOBAL uint8_t Zigbee_Rx_flag ;  //接收完成标志位

GLOBAL void Wifi_data_Receive( uint8_t res);
GLOBAL void Zigbee_data_Receive( uint8_t res);

void Timer4_Init(uint16_t arr,uint16_t psc);


#undef GLOBAL

#endif //__DATA_CHANNEL_H__


#ifndef __TBA_H
#define __TBA_H
#include "sys.h"
#include "stm32f4xx.h"

#define L_LED  0x01
#define R_LED  0x02

#define Tba_L_LED 	PHout(10)
#define Tba_R_LED 	PHout(11)


void Tba_Init(void);
uint8_t Get_tba_phsis_value(void);
void Set_tba_Beep(uint8_t swch);
void Set_tba_WheelLED(uint8_t LorR,uint8_t swch);

#endif


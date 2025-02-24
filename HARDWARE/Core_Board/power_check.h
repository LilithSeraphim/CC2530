
#ifndef __POWER_CHECK_H__
#define __POWER_CHECK_H__

#include "sys.h"
void Electricity_Init(void);
void Parameter_Init(void) ;
u16  Get_Electricity(uint8_t times );
void Power_Check(void);

extern uint8_t Electric_Buf[2];

#endif //__POWER_CHECK_H__




/*******************************************************************************
*
*       Copyright(c) 2008-2017; Beijing HeartCare Medical Co. LTD.
*
*       All rights reserved.  Protected by international copyright laws.
*       Knowledge of the source code may NOT be used to develop a similar product.
*
* File:          wang_pwm.h
* Compiler :     
* Revision :     Revision
* Date :         2017-01-07
* Updated by :   
* Description :  
*                
* 
* LPC11C14 sytem clock: 48Mhz
* system clock: 48MHz
********************************************************************************
* This edition can only be used in DCBoard V6.0 using MornSun DC/DC Module
*******************************************************************************/

#ifndef __WANG_PWM_H_
#define __WANG_PWM_H_

#include "board.h"

void Timer16_0_Init (void);
void T16B0_cnt_init(void);
void Timer32_1_Init (void);
void Timer32_0_Init (void);
void Timer16_1_Init(void);
void delay(uint32_t time);
void ADC_Read (uint32_t ulTime);
uint16_t ADCRead(uint8_t channlenum);

#endif

// end of the file

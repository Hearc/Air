/*******************************************************************************
*
*       Copyright(c) 2008-2017; Beijing HeartCare Medical Co. LTD.
*
*       All rights reserved.  Protected by international copyright laws.
*       Knowledge of the source code may NOT be used to develop a similar product.
*
* File:          wang_pwm.c
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


#include "wang_pwm.h"
uint8_t data[3];

/*********************************************************************************************************
**	序号	|BYTE1	|BYTE2	|BYTE3	|BYTE4	|BYTE5	|BYTE6	|BYTE7	|BYTE8
**	 	1		|	0xA0	|	运行	|		 无定义			|				错误标识 AIR_ERR1
**		2		|	0xB0	| 状态  |	硬件  | 软件	|			上电后时间				|
**		3		|	0xB1	|	运行	| 检测器灵敏度	|开,角度传感器值|关,角度传感器值
**
*********************************************************************************************************/

uint16_t CmpCounter;
/* 空气检测器灵敏度 */
uint16_t sensitivity = 0x46;
//主机运行状态	
uint8_t masterstate;
//气泡检测状态 有气泡为 0x01 ；没有气泡为 0x00 
uint8_t airbubblestart;
uint8_t	hardwareversion = 0x05,softwareversion = 0x05;
/*电机开关角度值*/
uint16_t anglevalue_on = 0,anglevalue_off = 0;
uint16_t anglevalue;                //测量电机角度
bool motor_on;
bool motor_off;

uint32_t movetime;                  //主机运行时间
uint8_t check;                      //00 切换到自检状态 01 下一步 02 系统维护	
uint8_t checkstart = 0;

/*********************************************************************************************************
  宏定义
*********************************************************************************************************/
#define    UART_BPS       115200                                        /* 串口通信波特率               */
/*********************************************************************************************************
  全局变量
*********************************************************************************************************/
volatile   uint8_t          GucRcvNew;                                  /* 串口接收新数据的标志         */
uint8_t      GucRcvBuf[10] ;                                            /* 串口接收数据缓冲区           */
uint32_t     GulNum;                                                    /* 串口接收数据的个数           */

uint8_t TxBuff[8];
uint8_t RxBuff[8];

/*! \fn       start_pwm(void)
 *  \brief    ??PWM
 *
 *  \param    None
 *  \return   None
 */
static void start_pwm(void)
{
    LPC_TIMER32_1->TCR = 0x01;
}

/*! \fn       stop_pwm(void)
 *  \brief    ??PWM
 *
 *  \param    None
 *  \return   None
 */
static void stop_pwm(void)
{
    //LPC_SCT->CTRL_U |= (1 << 3);
    LPC_TIMER32_1->TCR = 0x02;
}

/*****************************************************************************
 * Private functions
 ****************************************************************************/
/* 读取角度值 */ 
void Angle_Value()
{
	/* Start A/D conversion 开启ADC转换*/
//	Chip_ADC_EnableChannel(LPC_ADC, ADC_CH1, ENABLE);		//使能ADC通道1
//	Chip_ADC_EnableChannel(LPC_ADC, ADC_CH0, DISABLE);		//使能ADC通道0
	//Chip_ADC_SetStartMode(LPC_ADC, ADC_START_NOW, ADC_TRIGGERMODE_RISING);
	Chip_ADC_SetStartMode(LPC_ADC, ADC_NO_START, ADC_TRIGGERMODE_RISING);
	/* Waiting for A/D conversion complete等待转换完成 */
	while (Chip_ADC_ReadStatus(LPC_ADC, ADC_CH1, ADC_DR_DONE_STAT) != SET) {}
	/* Read ADC value */
	Chip_ADC_ReadValue(LPC_ADC, ADC_CH1, &anglevalue);
		
//	Chip_ADC_EnableChannel(LPC_ADC, ADC_CH0, ENABLE);		//使能ADC通道1
//	Chip_ADC_EnableChannel(LPC_ADC, ADC_CH1, DISABLE);		//使能ADC通道0
}	
 
/*********************************************************************************************************
** Function name:       uartSendByte
** Descriptions:        向串口发送子节数据，并等待数据发送完成，使用查询方式
** input parameters:    ucDat:   要发送的数据
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void uartSendByte (uint8_t ucDat)
{
    LPC_USART->THR = ucDat;                                              /*  写入数据                    */
    while ((LPC_USART->LSR & 0x40) == 0);                                /*  等待数据发送完毕            */
}

/*********************************************************************************************************
** Function name:       uartSendStr
** Descriptions:        向串口发送字符串
** input parameters:    pucStr:  要发送的字符串指针
**                      ulNum:   要发送的数据个数
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void uartSendStr (uint8_t const *pucStr, uint32_t ulNum)
{
    uint32_t i;

    for (i = 0; i < ulNum; i++){                                        /* 发送指定个字节数据           */
        uartSendByte (*pucStr++);
    }
}
/*********************************************************************************************************
* Function Name:        UART_IRQHandler
* Description:          UART中断服务函数
* Input:                无
* Output:               无
* Return:               无
*********************************************************************************************************/
void UART_IRQHandler (void)
{
    GulNum = 0;

    while ((LPC_USART->IIR & 0x01) == 0)
    {                                                                   /*  判断是否有中断挂起          */
        switch (LPC_USART->IIR & 0x0E){                                 /*  判断中断标志                */
        
            case 0x04:                                                  /*  接收数据中断                */
                GucRcvNew = 1;                                          /*  置接收新数据标志            */
                for (GulNum = 0; GulNum < 8; GulNum++){                 /*  连续接收8个字节             */
                GucRcvBuf[GulNum] = LPC_USART->RBR;
                }
                break;
            
            case 0x0C:                                                  /*  字符超时中断                */
                GucRcvNew = 1;
                while ((LPC_USART->LSR & 0x01) == 0x01){                /*  判断数据是否接收完毕        */
                    GucRcvBuf[GulNum] = LPC_USART->RBR;
                    GulNum++;
                }
                break;
                
            default:
                break;
        }
    }
    if(GucRcvNew == 1)                                                  /* 判断是否有新数据             */
    {
        GucRcvNew = 0;                                                  /* 清除标志                     */
        //uartSendStr(GucRcvBuf, GulNum);                               /* 向串口发送数据               */
        masterstate = GucRcvBuf[1];
        switch (GucRcvBuf[0])
        {
            case 0x50:
            {
                //masterstate = GucRcvBuf[1];
                TxBuff[0] = 0xB0;
                TxBuff[1] = masterstate;

                TxBuff[2] = hardwareversion;
                TxBuff[3] = softwareversion ;
                //上电运行时间
                TxBuff[4] = movetime >> 24;
                TxBuff[5] = (movetime & 0xFFFFFF) >> 16 ;
                TxBuff[6] = (movetime & 0xFFFF) >> 8;
                TxBuff[7] = movetime & 0xFF;
                Chip_UART_SendBlocking(LPC_USART,&TxBuff, 8);
                break;
            }

            case 0x51:
            {
                //masterstate = GucRcvBuf[1];
                TxBuff[0] = 0xB1;
                TxBuff[1] = masterstate;
                TxBuff[2] = sensitivity >> 8;
                TxBuff[3] = sensitivity &0xFF;
                TxBuff[4] = 0;
                TxBuff[5] = 0;
                TxBuff[6] = 0;
                TxBuff[7] = 0;
                Chip_UART_SendBlocking(LPC_USART,&TxBuff, 8);
                break;
            }

            case 0x61:
            {
                //masterstate = GucRcvBuf[1];
                sensitivity = (GucRcvBuf[2] << 8) + GucRcvBuf[3];
                break;
            }
           
            case 0x60:
            {
                //masterstate = GucRcvBuf[1];
                check = GucRcvBuf[8];
                if((masterstate == 0x02) && (check == 0x01 || check == 0x02))
                {
                    checkstart = 1;
                }
            }
            default:
                break;
        }
    }
}														

/*********************************************************************************************************
** Function name:       timer0Init
** Descriptions:        16位定时器0初始化函数
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void Timer16_0_Init (void)
{
    //配置PWM引脚
    Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO0_8, (IOCON_FUNC2 | IOCON_MODE_INACT)); /* CT16B0_MAT0 */
//	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO0_9, (IOCON_FUNC2 | IOCON_MODE_INACT)); /* CT16B0_MAT1 */

    Chip_TIMER_Init(LPC_TIMER16_0);                                 /* 打开定时器模块               */

    LPC_TIMER16_0->TCR     = 0x02;                                  /* 定时器复位                   */
    LPC_TIMER16_0->PR      = 0;                                     /* 设置分频系数                 */
    LPC_TIMER16_0->PWMC    = 0x03;                                  /* 设置MAT0，1 PWM输出          */
    LPC_TIMER16_0->MCR     = 0x02<<9;                               /* 设置MR3匹配后复位TC,PWM周期  */
    LPC_TIMER16_0->MR[3]     = SystemCoreClock / 10000;             /* 周期控制，1秒                */
    LPC_TIMER16_0->MR[0]     = LPC_TIMER16_0->MR[3]/2;              /* MAT0输出50%方波              */
//    LPC_TIMER16_0->MR[1]     = LPC_TIMER16_0->MR[3]/4;            /* MAT1输出75%方波              */
    LPC_TIMER16_0->TCR     = 0x01;                                  /* 启动定时器                   */
}

//输入捕获模式
void T16B0_cnt_init(void)
{
    Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO0_2, (IOCON_FUNC2 | IOCON_MODE_INACT));    /* CT16B0_MAT0 */
    Chip_TIMER_Init(LPC_TIMER16_0);        //使能时钟
    LPC_TIMER16_0->TCR     = 0x02;         //复位定时器
    LPC_TIMER16_0->CTCR = 0x01;            //计数器模式 引脚CT16B_CAP0 上升沿计数
    LPC_TIMER16_0->TCR     = 0x01;         //启动定时器
}
/*********************************************************************************************************
** Function name:       timer1Init
** Descriptions:        32位定时器1初始化函数
** input parameters:    无
** output parameters:   无
** Returned value:      无
** P1_1输出2MHz方波
*********************************************************************************************************/
void Timer32_1_Init (void)
{
    //配置PWM引脚
    Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_1, (IOCON_FUNC3 | IOCON_MODE_INACT)); /* CT32B1_MAT0 */

    Chip_TIMER_Init(LPC_TIMER32_1);                                 /* 打开定时器模块               */
//		Chip_TIMER_Reset(LPC_TIMER32_1);

    LPC_TIMER32_1->TCR     = 0x02;                                          /* 定时器复位                   */
    LPC_TIMER32_1->PR      = 0;                                             /* 设置分频系数                 */
    LPC_TIMER32_1->PWMC    = 0x01;                                          /* 设置MAT0，1PWM输出           */
    LPC_TIMER32_1->MCR     = 0x02<<9;                                       /* 设置MR3匹配后复位TC,PWM周期  */
    LPC_TIMER32_1->MR[3]     = Chip_Clock_GetSystemClockRate() /1500000;    /* 周期控制，1秒                */
    LPC_TIMER32_1->MR[0]     = LPC_TIMER32_1->MR[3]/2;                      /* MAT0输出50%方波              */
//    Chip_TIMER_SetMatch(LPC_TIMER32_1, 1, (Chip_Clock_GetSystemClockRate() / 1000000));
//    LPC_TIMER32_1->MR[1]     = LPC_TIMER32_1->MR[3]/4;                    /* MAT1输出75%方波              */
    LPC_TIMER32_1->TCR     = 0x01;                                          /* 启动定时器                   */
    
//		Chip_TIMER_Enable(LPC_TIMER32_1);
//		NVIC_ClearPendingIRQ(TIMER_32_1_IRQn);  			// Enable timer interrupt
//		NVIC_EnableIRQ(TIMER_32_1_IRQn);
}

/*********************************************************************************************************
** Function name:       timer0Init
** Descriptions:        32位定时器1初始化函数
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void Timer32_0_Init(void)
{
    uint32_t timerFreq;
    
    Chip_TIMER_Init(LPC_TIMER32_0);                                 // Enable timer 1 clock 
    timerFreq = Chip_Clock_GetSystemClockRate();                    // Timer rate is system clock rate 
    Chip_TIMER_Reset(LPC_TIMER32_0);                                // Timer setup for match and interrupt at TICKRATE_HZ
    Chip_TIMER_MatchEnableInt(LPC_TIMER32_0, 1U);
    
    Chip_TIMER_SetMatch(LPC_TIMER32_0, 1U, (timerFreq / 10000));    //100us中断 10KHz
    
    Chip_TIMER_ResetOnMatchEnable(LPC_TIMER32_0, 1U);
    __NOP(); __NOP(); __NOP();
    
    Chip_TIMER_Enable(LPC_TIMER32_0);
    NVIC_ClearPendingIRQ(TIMER_32_0_IRQn);                          // Enable timer interrupt
    NVIC_EnableIRQ(TIMER_32_0_IRQn);
}

/*********************************************************************************************************
** Function name:       timer0Init
** Descriptions:        16位定时器1初始化函数
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void Timer16_1_Init(void)
{
    uint32_t timerFreq;
    
    Chip_TIMER_Init(LPC_TIMER16_1);                         // Enable timer 1 clock 
    timerFreq = Chip_Clock_GetSystemClockRate();            // Timer rate is system clock rate 
    Chip_TIMER_Reset(LPC_TIMER16_1);                        // Timer setup for match and interrupt at TICKRATE_HZ
    Chip_TIMER_MatchEnableInt(LPC_TIMER16_1, 1U);
    
    Chip_TIMER_SetMatch(LPC_TIMER16_1, 1U, (timerFreq / 10000));   //1ms中断 10KHz
    
    Chip_TIMER_ResetOnMatchEnable(LPC_TIMER16_1, 1U);
    __NOP(); __NOP(); __NOP();
    
    Chip_TIMER_Enable(LPC_TIMER16_1);
    NVIC_ClearPendingIRQ(TIMER_16_1_IRQn);                         // Enable timer interrupt
    NVIC_EnableIRQ(TIMER_16_1_IRQn);
}

/**********************************************************************
 * @brief	  Handle interrupt from 32-bit timer
 * @return	Nothing
 * @note	  used for 
 * @author  FENG XQ
 * @date    2016-08-13
 *********************************************************************/
void TIMER32_0_IRQHandler(void)
{
    if (Chip_TIMER_MatchPending(LPC_TIMER32_0, 1U)) 
    {
        Chip_TIMER_ClearMatch(LPC_TIMER32_0, 1U);
        start_pwm();                //开启PWM向压电陶瓷发送2MHz方波
        delay(4);
        stop_pwm();                 //停止PWM停止发送方波
//        Chip_GPIO_SetPinToggle(LPC_GPIO, 1, 8);

        ADC_Read(1);                //读取压电陶瓷反馈电压
    }
}


void TIMER16_1_IRQHandler(void)
{
    if (Chip_TIMER_MatchPending(LPC_TIMER16_1, 1U))
    {
        Chip_GPIO_SetPinToggle(LPC_GPIO, 2, 1);
    }
}

void delay(uint32_t time)
{
    uint32_t i;
    while(time--)
    {
        for(i = 0;i < 11 ;i++);
    }
}

/*********************************************************************************************************
** Function name:       ADC_Read
** Descriptions:        AD采值
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void ADC_Read (uint32_t ulTime)
{
    uint16_t dataADC;

//    while (ulTime--) {
    /* Start A/D conversion 开启ADC转换*/
    Chip_ADC_SetStartMode(LPC_ADC, ADC_START_NOW, ADC_TRIGGERMODE_RISING);
    //Chip_ADC_SetStartMode(LPC_ADC, ADC_NO_START, ADC_TRIGGERMODE_RISING);
    
    /* Waiting for A/D conversion complete等待转换完成 */
    while (Chip_ADC_ReadStatus(LPC_ADC, ADC_CH0, ADC_DR_DONE_STAT) != SET) {}
//    Chip_GPIO_WritePortBit(LPC_GPIO,2,1,0);
    /* Read ADC value */
    Chip_ADC_ReadValue(LPC_ADC, ADC_CH0, &dataADC);

    if(dataADC > 80)
    {
        CmpCounter++;
    }
}

/**********************************************************************
 * @brief	SysTick_Handler Interrupt Handler
 * @return	Nothing
 * @note	Th.
 * @author  FENG XQ
 * @date    2016-08-16
 *********************************************************************/
void SysTick_Handler(void)
{
    Chip_GPIO_SetPinToggle(LPC_GPIO, 2, 1);

    if(CmpCounter > sensitivity)                        /* 正常时 */
    {
        Chip_GPIO_WritePortBit(LPC_GPIO,2,7,0);
        /*  */
        airbubblestart = 0;
        //判断电机是否开启 如果电机没有开启 motor_on = 1;
        if(anglevalue < anglevalue_on)
        {
            motor_on = 1; 
        }
    }
    else
    {
        Chip_GPIO_WritePortBit(LPC_GPIO,2,7,1);
        airbubblestart = 1;
        //判断电机是否关闭 如果未关闭 motor_off = 1;
        if(anglevalue > anglevalue_off)
            motor_off = 1;
    }

    CmpCounter = 0;
    if(checkstart)
    {
        checkstart = 0;
        if(airbubblestart)
        {
            TxBuff[0] = 0xA0;
            TxBuff[1] = masterstate;
            TxBuff[2] = 0;
            TxBuff[3] = 0;
            TxBuff[4] = 1;
            TxBuff[5] = 0;
            TxBuff[6] = 0;
            TxBuff[7] = 0;
            Chip_UART_SendBlocking(LPC_USART,&TxBuff, 8);
        }
    }

    if(airbubblestart == 1)
    {
        if(motor_off)
        {
            motor_off = 0;
            /* 电机控制 关 */


            TxBuff[0] = 0xA0;
            TxBuff[1] = masterstate;
            TxBuff[2] = 3;
            TxBuff[3] = 0;
            TxBuff[4] = 1;
            TxBuff[5] = 0;
            TxBuff[6] = 0;
            TxBuff[7] = 0;
            Chip_UART_SendBlocking(LPC_USART,&TxBuff, 8);
            Chip_GPIO_WritePortBit(LPC_GPIO,2,7,1);
            //Chip_GPIO_WritePortBit(LPC_GPIO,2,10,0);		//使能电机
        }
    }

    if(airbubblestart == 0)
    {
        if(motor_on)
        {
            motor_on = 0;
            /* 电机控制 开 */
            Chip_GPIO_WritePortBit(LPC_GPIO,2,7,1);
            //Chip_GPIO_WritePortBit(LPC_GPIO,2,10,0);		//使能电机
        }
    }
    /* 电机保持原有状态 */
}

uint16_t ADCRead(uint8_t channlenum)
{
    uint32_t regVal = 0;
    uint16_t ADC_Data = 0;
//	if(channlenum >= 8)
//		channlenum = 0;
    //LPC_ADC->CR |= 1 << 24;
    //等待转换完成
    while((LPC_ADC->DR[channlenum] & 0x80000000) == 0);

    regVal = LPC_ADC->DR[channlenum];
    if(regVal & ADC_DR_OVERRUN_STAT)
    {
        return (0);
    }
    ADC_Data = regVal & 0x3FF;

    return(ADC_Data);
}

//end of the file

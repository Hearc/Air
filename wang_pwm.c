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
**	���	|BYTE1	|BYTE2	|BYTE3	|BYTE4	|BYTE5	|BYTE6	|BYTE7	|BYTE8
**	 	1		|	0xA0	|	����	|		 �޶���			|				�����ʶ AIR_ERR1
**		2		|	0xB0	| ״̬  |	Ӳ��  | ���	|			�ϵ��ʱ��				|
**		3		|	0xB1	|	����	| �����������	|��,�Ƕȴ�����ֵ|��,�Ƕȴ�����ֵ
**
*********************************************************************************************************/

uint16_t CmpCounter;
/* ��������������� */
uint16_t sensitivity = 0x46;
//��������״̬	
uint8_t masterstate;
//���ݼ��״̬ ������Ϊ 0x01 ��û������Ϊ 0x00 
uint8_t airbubblestart;
uint8_t	hardwareversion = 0x05,softwareversion = 0x05;
/*������ؽǶ�ֵ*/
uint16_t anglevalue_on = 0,anglevalue_off = 0;
uint16_t anglevalue;                //��������Ƕ�
bool motor_on;
bool motor_off;

uint32_t movetime;                  //��������ʱ��
uint8_t check;                      //00 �л����Լ�״̬ 01 ��һ�� 02 ϵͳά��	
uint8_t checkstart = 0;

/*********************************************************************************************************
  �궨��
*********************************************************************************************************/
#define    UART_BPS       115200                                        /* ����ͨ�Ų�����               */
/*********************************************************************************************************
  ȫ�ֱ���
*********************************************************************************************************/
volatile   uint8_t          GucRcvNew;                                  /* ���ڽ��������ݵı�־         */
uint8_t      GucRcvBuf[10] ;                                            /* ���ڽ������ݻ�����           */
uint32_t     GulNum;                                                    /* ���ڽ������ݵĸ���           */

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
/* ��ȡ�Ƕ�ֵ */ 
void Angle_Value()
{
	/* Start A/D conversion ����ADCת��*/
//	Chip_ADC_EnableChannel(LPC_ADC, ADC_CH1, ENABLE);		//ʹ��ADCͨ��1
//	Chip_ADC_EnableChannel(LPC_ADC, ADC_CH0, DISABLE);		//ʹ��ADCͨ��0
	//Chip_ADC_SetStartMode(LPC_ADC, ADC_START_NOW, ADC_TRIGGERMODE_RISING);
	Chip_ADC_SetStartMode(LPC_ADC, ADC_NO_START, ADC_TRIGGERMODE_RISING);
	/* Waiting for A/D conversion complete�ȴ�ת����� */
	while (Chip_ADC_ReadStatus(LPC_ADC, ADC_CH1, ADC_DR_DONE_STAT) != SET) {}
	/* Read ADC value */
	Chip_ADC_ReadValue(LPC_ADC, ADC_CH1, &anglevalue);
		
//	Chip_ADC_EnableChannel(LPC_ADC, ADC_CH0, ENABLE);		//ʹ��ADCͨ��1
//	Chip_ADC_EnableChannel(LPC_ADC, ADC_CH1, DISABLE);		//ʹ��ADCͨ��0
}	
 
/*********************************************************************************************************
** Function name:       uartSendByte
** Descriptions:        �򴮿ڷ����ӽ����ݣ����ȴ����ݷ�����ɣ�ʹ�ò�ѯ��ʽ
** input parameters:    ucDat:   Ҫ���͵�����
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void uartSendByte (uint8_t ucDat)
{
    LPC_USART->THR = ucDat;                                              /*  д������                    */
    while ((LPC_USART->LSR & 0x40) == 0);                                /*  �ȴ����ݷ������            */
}

/*********************************************************************************************************
** Function name:       uartSendStr
** Descriptions:        �򴮿ڷ����ַ���
** input parameters:    pucStr:  Ҫ���͵��ַ���ָ��
**                      ulNum:   Ҫ���͵����ݸ���
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void uartSendStr (uint8_t const *pucStr, uint32_t ulNum)
{
    uint32_t i;

    for (i = 0; i < ulNum; i++){                                        /* ����ָ�����ֽ�����           */
        uartSendByte (*pucStr++);
    }
}
/*********************************************************************************************************
* Function Name:        UART_IRQHandler
* Description:          UART�жϷ�����
* Input:                ��
* Output:               ��
* Return:               ��
*********************************************************************************************************/
void UART_IRQHandler (void)
{
    GulNum = 0;

    while ((LPC_USART->IIR & 0x01) == 0)
    {                                                                   /*  �ж��Ƿ����жϹ���          */
        switch (LPC_USART->IIR & 0x0E){                                 /*  �ж��жϱ�־                */
        
            case 0x04:                                                  /*  ���������ж�                */
                GucRcvNew = 1;                                          /*  �ý��������ݱ�־            */
                for (GulNum = 0; GulNum < 8; GulNum++){                 /*  ��������8���ֽ�             */
                GucRcvBuf[GulNum] = LPC_USART->RBR;
                }
                break;
            
            case 0x0C:                                                  /*  �ַ���ʱ�ж�                */
                GucRcvNew = 1;
                while ((LPC_USART->LSR & 0x01) == 0x01){                /*  �ж������Ƿ�������        */
                    GucRcvBuf[GulNum] = LPC_USART->RBR;
                    GulNum++;
                }
                break;
                
            default:
                break;
        }
    }
    if(GucRcvNew == 1)                                                  /* �ж��Ƿ���������             */
    {
        GucRcvNew = 0;                                                  /* �����־                     */
        //uartSendStr(GucRcvBuf, GulNum);                               /* �򴮿ڷ�������               */
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
                //�ϵ�����ʱ��
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
** Descriptions:        16λ��ʱ��0��ʼ������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void Timer16_0_Init (void)
{
    //����PWM����
    Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO0_8, (IOCON_FUNC2 | IOCON_MODE_INACT)); /* CT16B0_MAT0 */
//	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO0_9, (IOCON_FUNC2 | IOCON_MODE_INACT)); /* CT16B0_MAT1 */

    Chip_TIMER_Init(LPC_TIMER16_0);                                 /* �򿪶�ʱ��ģ��               */

    LPC_TIMER16_0->TCR     = 0x02;                                  /* ��ʱ����λ                   */
    LPC_TIMER16_0->PR      = 0;                                     /* ���÷�Ƶϵ��                 */
    LPC_TIMER16_0->PWMC    = 0x03;                                  /* ����MAT0��1 PWM���          */
    LPC_TIMER16_0->MCR     = 0x02<<9;                               /* ����MR3ƥ���λTC,PWM����  */
    LPC_TIMER16_0->MR[3]     = SystemCoreClock / 10000;             /* ���ڿ��ƣ�1��                */
    LPC_TIMER16_0->MR[0]     = LPC_TIMER16_0->MR[3]/2;              /* MAT0���50%����              */
//    LPC_TIMER16_0->MR[1]     = LPC_TIMER16_0->MR[3]/4;            /* MAT1���75%����              */
    LPC_TIMER16_0->TCR     = 0x01;                                  /* ������ʱ��                   */
}

//���벶��ģʽ
void T16B0_cnt_init(void)
{
    Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO0_2, (IOCON_FUNC2 | IOCON_MODE_INACT));    /* CT16B0_MAT0 */
    Chip_TIMER_Init(LPC_TIMER16_0);        //ʹ��ʱ��
    LPC_TIMER16_0->TCR     = 0x02;         //��λ��ʱ��
    LPC_TIMER16_0->CTCR = 0x01;            //������ģʽ ����CT16B_CAP0 �����ؼ���
    LPC_TIMER16_0->TCR     = 0x01;         //������ʱ��
}
/*********************************************************************************************************
** Function name:       timer1Init
** Descriptions:        32λ��ʱ��1��ʼ������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
** P1_1���2MHz����
*********************************************************************************************************/
void Timer32_1_Init (void)
{
    //����PWM����
    Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_1, (IOCON_FUNC3 | IOCON_MODE_INACT)); /* CT32B1_MAT0 */

    Chip_TIMER_Init(LPC_TIMER32_1);                                 /* �򿪶�ʱ��ģ��               */
//		Chip_TIMER_Reset(LPC_TIMER32_1);

    LPC_TIMER32_1->TCR     = 0x02;                                          /* ��ʱ����λ                   */
    LPC_TIMER32_1->PR      = 0;                                             /* ���÷�Ƶϵ��                 */
    LPC_TIMER32_1->PWMC    = 0x01;                                          /* ����MAT0��1PWM���           */
    LPC_TIMER32_1->MCR     = 0x02<<9;                                       /* ����MR3ƥ���λTC,PWM����  */
    LPC_TIMER32_1->MR[3]     = Chip_Clock_GetSystemClockRate() /1500000;    /* ���ڿ��ƣ�1��                */
    LPC_TIMER32_1->MR[0]     = LPC_TIMER32_1->MR[3]/2;                      /* MAT0���50%����              */
//    Chip_TIMER_SetMatch(LPC_TIMER32_1, 1, (Chip_Clock_GetSystemClockRate() / 1000000));
//    LPC_TIMER32_1->MR[1]     = LPC_TIMER32_1->MR[3]/4;                    /* MAT1���75%����              */
    LPC_TIMER32_1->TCR     = 0x01;                                          /* ������ʱ��                   */
    
//		Chip_TIMER_Enable(LPC_TIMER32_1);
//		NVIC_ClearPendingIRQ(TIMER_32_1_IRQn);  			// Enable timer interrupt
//		NVIC_EnableIRQ(TIMER_32_1_IRQn);
}

/*********************************************************************************************************
** Function name:       timer0Init
** Descriptions:        32λ��ʱ��1��ʼ������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void Timer32_0_Init(void)
{
    uint32_t timerFreq;
    
    Chip_TIMER_Init(LPC_TIMER32_0);                                 // Enable timer 1 clock 
    timerFreq = Chip_Clock_GetSystemClockRate();                    // Timer rate is system clock rate 
    Chip_TIMER_Reset(LPC_TIMER32_0);                                // Timer setup for match and interrupt at TICKRATE_HZ
    Chip_TIMER_MatchEnableInt(LPC_TIMER32_0, 1U);
    
    Chip_TIMER_SetMatch(LPC_TIMER32_0, 1U, (timerFreq / 10000));    //100us�ж� 10KHz
    
    Chip_TIMER_ResetOnMatchEnable(LPC_TIMER32_0, 1U);
    __NOP(); __NOP(); __NOP();
    
    Chip_TIMER_Enable(LPC_TIMER32_0);
    NVIC_ClearPendingIRQ(TIMER_32_0_IRQn);                          // Enable timer interrupt
    NVIC_EnableIRQ(TIMER_32_0_IRQn);
}

/*********************************************************************************************************
** Function name:       timer0Init
** Descriptions:        16λ��ʱ��1��ʼ������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void Timer16_1_Init(void)
{
    uint32_t timerFreq;
    
    Chip_TIMER_Init(LPC_TIMER16_1);                         // Enable timer 1 clock 
    timerFreq = Chip_Clock_GetSystemClockRate();            // Timer rate is system clock rate 
    Chip_TIMER_Reset(LPC_TIMER16_1);                        // Timer setup for match and interrupt at TICKRATE_HZ
    Chip_TIMER_MatchEnableInt(LPC_TIMER16_1, 1U);
    
    Chip_TIMER_SetMatch(LPC_TIMER16_1, 1U, (timerFreq / 10000));   //1ms�ж� 10KHz
    
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
        start_pwm();                //����PWM��ѹ���մɷ���2MHz����
        delay(4);
        stop_pwm();                 //ֹͣPWMֹͣ���ͷ���
//        Chip_GPIO_SetPinToggle(LPC_GPIO, 1, 8);

        ADC_Read(1);                //��ȡѹ���մɷ�����ѹ
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
** Descriptions:        AD��ֵ
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void ADC_Read (uint32_t ulTime)
{
    uint16_t dataADC;

//    while (ulTime--) {
    /* Start A/D conversion ����ADCת��*/
    Chip_ADC_SetStartMode(LPC_ADC, ADC_START_NOW, ADC_TRIGGERMODE_RISING);
    //Chip_ADC_SetStartMode(LPC_ADC, ADC_NO_START, ADC_TRIGGERMODE_RISING);
    
    /* Waiting for A/D conversion complete�ȴ�ת����� */
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

    if(CmpCounter > sensitivity)                        /* ����ʱ */
    {
        Chip_GPIO_WritePortBit(LPC_GPIO,2,7,0);
        /*  */
        airbubblestart = 0;
        //�жϵ���Ƿ��� ������û�п��� motor_on = 1;
        if(anglevalue < anglevalue_on)
        {
            motor_on = 1; 
        }
    }
    else
    {
        Chip_GPIO_WritePortBit(LPC_GPIO,2,7,1);
        airbubblestart = 1;
        //�жϵ���Ƿ�ر� ���δ�ر� motor_off = 1;
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
            /* ������� �� */


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
            //Chip_GPIO_WritePortBit(LPC_GPIO,2,10,0);		//ʹ�ܵ��
        }
    }

    if(airbubblestart == 0)
    {
        if(motor_on)
        {
            motor_on = 0;
            /* ������� �� */
            Chip_GPIO_WritePortBit(LPC_GPIO,2,7,1);
            //Chip_GPIO_WritePortBit(LPC_GPIO,2,10,0);		//ʹ�ܵ��
        }
    }
    /* �������ԭ��״̬ */
}

uint16_t ADCRead(uint8_t channlenum)
{
    uint32_t regVal = 0;
    uint16_t ADC_Data = 0;
//	if(channlenum >= 8)
//		channlenum = 0;
    //LPC_ADC->CR |= 1 << 24;
    //�ȴ�ת�����
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

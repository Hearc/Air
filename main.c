/*******************************************************************************
*
*       Copyright(c) 2008-2017; Beijing HeartCare Medical Co. LTD.
*
*       All rights reserved.  Protected by international copyright laws.
*       Knowledge of the source code may NOT be used to develop a similar product.
*
* File:          main.c
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

// LPC11C24: Pin17=VDD_CAN  Pin18=CANL   Pin19=CANH   Pin20=VCC    Pin21=GND    Pin22=STB    Pin23=PIO0-6
// LPC11C14: Pin17=PIO1-9   Pin18=PIO2-4 Pin19=CAN-RX Pin20=CAN-TX Pin21=PIO2-5 Pin22=PIO0-6 Pin203=PIO0-7
// LED = PIO2-7
// TinyM0-CAN: UART JP4  PIO1_6 = RXD   PIO1_7 = TXD

#include "wang_pwm.h"
#include "feng_can.h"

#define TIMER_TICKRATE_IN_HZ    10UL                //
#define SYSTICK_RATE_IN_HZ      3000UL              //

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/
volatile uint8_t initialized;
uint32_t USecPerOverflow = 65536 * 2;
static ADC_CLOCK_SETUP_T ADCSetup;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/
static void Init_ADC_PinMux(void)
{
#if (defined(BOARD_NXP_XPRESSO_11U14) || defined(BOARD_NGX_BLUEBOARD_11U24) || defined(BOARD_NXP_XPRESSO_11U37H))
    Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 11, FUNC2);
#elif defined(BOARD_NXP_XPRESSO_11C24)
    Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO0_11, FUNC2);	
    Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_0, FUNC2);
#else
    #error "Pin muxing for ADC not configured"
#endif
}

/*****************************************************************************
 * 函数原型声明
 ****************************************************************************/
void SysTick_Handler(void);
void TIMER32_0_IRQHandler(void);
void myDelay (uint32_t ulTime);

static void fail(void)
{
    while (1) { }
}

void init(void)
{
    //CriticalSectionLocker lock;
    if (!initialized)
    {
        initialized = true;

        if ((SystemCoreClock % 1000000) != 0)  // Core clock frequency validation
        {
            fail();
        }

        if (SysTick_Config((SystemCoreClock / 1000000) * USecPerOverflow) != 0)
        {
            fail();
        }
    }
}

/**********************************************************************
 * @brief	  Main routine for lpc11c14 CCAN_ROM application
 * @return	Nothing
 * @note	  Function  
 * @author  FENG XQ
 * @date    2016-08-16
 *********************************************************************/
int main(void)
{
    SystemCoreClockUpdate();
    Board_Init();
    //开启串口中断
    NVIC_EnableIRQ(UART0_IRQn);                              /* 使能UART中断，并配置优先级   */
    NVIC_SetPriority(UART0_IRQn, 1);
    Chip_UART_IntEnable(LPC_USART,UART_IER_RBRINT);

    printf("SystemCoreClock = %d ! \r\n",SystemCoreClock);
    printf("System start! \r\n");

    Init_ADC_PinMux();                                       //ADC引脚配置 P0_11

    /* ADC Init */
    Chip_ADC_Init(LPC_ADC, &ADCSetup);                       //初始化ADC

    Chip_ADC_EnableChannel(LPC_ADC, ADC_CH0, ENABLE);        //使能ADC通道0

    /*配置引脚P1_1 产生2MHz PWM 发送到压电陶瓷*/
    Timer32_1_Init();

    /*每 100us 产生中断 向压电陶瓷发送方波 并读取反馈*/
    Timer32_0_Init();
    
    SysTick_Config(SystemCoreClock / 100);                  //200ms
    
    Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_IOCON);      //关闭引脚配置时钟
    
    while (1) 
    {
        __WFI();        // Go to Sleep
    }
}

//end of the file FENGXQ

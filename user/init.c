/*********************************************************/
/*
 * @demo
 * @固件库：超核V2.4
 * @author：袁双杰
 * @2018.10
 * @for seu2018 光电
 */
/*********************************************************/

#include "init.h"
#include "varieble.h"
#include "isr.h"
#include "sysinit.h"
#include "DEV_SCCB.h"
#include "oled.h"
#include "uart.h"
/* 初始化 */
void init( void )
{
        FTM_PWM_QuickInit(FTM0_CH4_PD04, kPWM_EdgeAligned, 3000,0);
        FTM_PWM_QuickInit(FTM0_CH5_PD05, kPWM_EdgeAligned, 3000,0);
        FTM_PWM_QuickInit(FTM0_CH6_PD06, kPWM_EdgeAligned, 3000,0);
        FTM_PWM_QuickInit(FTM0_CH7_PD07, kPWM_EdgeAligned, 3000,0);
        //GPIO_QuickInit(HW_GPIOB, 9 , kGPIO_Mode_OPP); //蜂鸣器
        GPIO_QuickInit(HW_GPIOB, 20, kGPIO_Mode_IFT);	//SW1初始化
        GPIO_QuickInit(HW_GPIOB, 21, kGPIO_Mode_IFT);	//SW2初始化
        GPIO_QuickInit(HW_GPIOB, 22, kGPIO_Mode_IFT);	//SW3初始化
        GPIO_QuickInit(HW_GPIOB, 23, kGPIO_Mode_IFT);	//SW4初始化
	/* 场中断 */
	GPIO_QuickInit( HW_GPIOC, 6, kGPIO_Mode_IPU );
	GPIO_CallbackInstall( HW_GPIOC, GPIO_ISR );
	GPIO_ITDMAConfig( HW_GPIOC, 6, kGPIO_IT_FallingEdge, false );   /*上升沿沿触发 */
	/* 行中断 */
	GPIO_QuickInit( HW_GPIOC, 7, kGPIO_Mode_IPD );
	GPIO_CallbackInstall( HW_GPIOC, GPIO_ISR );
	GPIO_ITDMAConfig( HW_GPIOC, 7, kGPIO_IT_RisingEdge, false );    /*下降沿触发 */
	/* PCLK */
	GPIO_QuickInit( HW_GPIOC, 2, kGPIO_Mode_IPU );
	GPIO_ITDMAConfig( HW_GPIOC, 2, kGPIO_DMA_RisingEdge, false );
	/* 摄像头数据口，下拉 */
	GPIO_QuickInit( HW_GPIOC, 8, kGPIO_Mode_IPD );
	GPIO_QuickInit( HW_GPIOC, 9, kGPIO_Mode_IPD );
	GPIO_QuickInit( HW_GPIOC, 10, kGPIO_Mode_IPD );
	GPIO_QuickInit( HW_GPIOC, 11, kGPIO_Mode_IPD );
	GPIO_QuickInit( HW_GPIOC, 12, kGPIO_Mode_IPD );
	GPIO_QuickInit( HW_GPIOC, 13, kGPIO_Mode_IPD );
	GPIO_QuickInit( HW_GPIOC, 14, kGPIO_Mode_IPD );
	GPIO_QuickInit( HW_GPIOC, 15, kGPIO_Mode_IPD );
        
        GPIO_QuickInit(HW_GPIOB, 16, kGPIO_Mode_IFT); // BUTTON1 
        GPIO_QuickInit(HW_GPIOB, 10, kGPIO_Mode_IFT); // BUTTON2 
        
        FTM_QD_QuickInit(FTM1_QD_PHA_PB00_PHB_PB01, kFTM_QD_NormalPolarity, kQD_PHABEncoding);//AB相编码器1初始化   P3
        
        LPTMR_PC_QuickInit(LPTMR_ALT2_PC05);//编码器2初始化   P5
        GPIO_QuickInit(HW_GPIOC, 4, kGPIO_Mode_IFT); //编码器2初始化 IO口读方向  P5
        
        //adc
        ADC_QuickInit(ADC1_SE4B_PC8, kADC_SingleDiff12or13);
        
        
	/* LED */
	GPIO_QuickInit( HW_GPIOC, 4, kGPIO_Mode_OPP );
	GPIO_QuickInit( HW_GPIOC, 5, kGPIO_Mode_OPP );
	GPIO_QuickInit( HW_GPIOB, 17, kGPIO_Mode_OPP );

	/* OLED IO初始化 */
	GPIO_QuickInit( HW_GPIOA, 12, kGPIO_Mode_OPP );
	GPIO_QuickInit( HW_GPIOA, 13, kGPIO_Mode_OPP );
	GPIO_QuickInit( HW_GPIOA, 14, kGPIO_Mode_OPP );
	GPIO_QuickInit( HW_GPIOA, 15, kGPIO_Mode_OPP );
	GPIO_QuickInit( HW_GPIOA, 16, kGPIO_Mode_OPP );
        GPIO_QuickInit( HW_GPIOE, 25, kGPIO_Mode_OPP );

	UART_QuickInit( UART3_RX_PC16_TX_PC17, 9600 );                /* 串口 */
        //UART_CallbackRxInstall(HW_UART3,UART3_RX_IRQHandler);//蓝牙接收中断
        //UART_ITDMAConfig(HW_UART3, kUART_IT_Rx, true);//蓝牙接收中断
	init_ov7620();                                                  /* 摄像头初始化 */
	
        OLED_Init();                                                    /* OLED初始化 */

        SYSTICK_DelayInit();
	init_steer();                                                   /* 舵机初始化 */
	GPIO_ITDMAConfig( HW_GPIOC, 6, kGPIO_IT_FallingEdge, true );
	GPIO_ITDMAConfig( HW_GPIOC, 7, kGPIO_IT_RisingEdge, true );
	GPIO_ITDMAConfig( HW_GPIOC, 2, kGPIO_DMA_RisingEdge, true );
        
	/* 五向开关初始化 */
	GPIO_QuickInit( HW_GPIOE, 0, kGPIO_Mode_IPU );                  /* 右 */
	GPIO_QuickInit( HW_GPIOE, 1, kGPIO_Mode_IPU );                  /*上 */
	GPIO_QuickInit( HW_GPIOE, 3, kGPIO_Mode_IPU );                  /* 左 */
	GPIO_QuickInit( HW_GPIOE, 2, kGPIO_Mode_IPU );                  /* 中 */
	GPIO_QuickInit( HW_GPIOC, 18, kGPIO_Mode_IPU );                 /*下 */
        
        /*驱动使能*/
        GPIO_WriteBit(HW_GPIOE, 25, 1); 
}


void init_ov7620( void )
{
	/* DMA初始化 */
	DMA_InitTypeDef DMA_InitStruct1 = { 0 };                        /* 定义初始化结构体 */
	DMA_InitStruct1.chl			= HW_DMA_CH0;
	DMA_InitStruct1.chlTriggerSource	= PORTC_DMAREQ;
	DMA_InitStruct1.triggerSourceMode	= kDMA_TriggerSource_Normal;
	DMA_InitStruct1.minorLoopByteCnt	= 1;
	DMA_InitStruct1.majorLoopCnt		= col_num;              /* 主循环采集数，即一次主循环采集一行 */

	DMA_InitStruct1.sAddr		= (uint32_t) &(PTC->PDIR) + 1;  /* dma源地址：ptc8~15 */
	DMA_InitStruct1.sLastAddrAdj	= 0;
	DMA_InitStruct1.sAddrOffset	= 0;
	DMA_InitStruct1.sDataWidth	= kDMA_DataWidthBit_8;          /* 数据宽度 */
	DMA_InitStruct1.sMod		= kDMA_ModuloDisable;

	DMA_InitStruct1.dLastAddrAdj	= 0;
	DMA_InitStruct1.dAddrOffset	= 1;
	DMA_InitStruct1.dDataWidth	= kDMA_DataWidthBit_8;
	DMA_InitStruct1.dMod		= kDMA_ModuloDisable;

	DMA_Init( &DMA_InitStruct1 );
	DMA_DisableRequest( HW_DMA_CH0 ); /* 先关闭DMA传输 */

	/* 配置摄像头寄存器 */
	uint8_t i = 0;


	/*
	 * 初始化SCCB所需引脚，SCCB是一种简化的类似于I2C的通信协议。
	 * 初始化PTC3和PTC0，作为SCCB中的SDA和SCL。
	 */
	GPIO_QuickInit( HW_GPIOC, 0, kGPIO_Mode_OPP );
	GPIO_QuickInit( HW_GPIOC, 3, kGPIO_Mode_OPP );
	while ( i == 0 )
		i += LPLD_SCCB_WriteReg( 0x14, 0x24 );  /* QVGA(320*120) */
	while ( i == 1 )
		i += LPLD_SCCB_WriteReg( 0x70, 0xc1 );  /* 驱动电流增加一倍 */
	while ( i == 2 )
		i += LPLD_SCCB_WriteReg( 0x24, 0x20 );  /* 连续采集模式(320*240) */
	while ( i == 3 )
		i += LPLD_SCCB_WriteReg( 0x06, 0xa0 );  /* 亮度控制 */
}


void init_steer( void )
{
	/*
	 * 占空比 = pwmDuty/10000*100%
	 * 理论中值占空比=1.5/20=7.5%,实际需根据机械结构和舵机本身进行调整
	 */
	FTM_PWM_QuickInit(FTM2_CH0_PB18, kPWM_EdgeAligned, 80, 1200);
	/* FTM_PWM_InvertPolarity(HW_FTM2,HW_FTM_CH1,kFTM_PWM_HighTrue); */
}
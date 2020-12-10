/*********************************************************/
/*
 * @demo
 * @�̼��⣺����V2.4
 * @author��Ԭ˫��
 * @2018.10
 * @for seu2018 ���
 */
/*********************************************************/

#include "init.h"
#include "varieble.h"
#include "isr.h"
#include "sysinit.h"
#include "DEV_SCCB.h"
#include "oled.h"
/* ��ʼ�� */
void init( void )
{
        FTM_PWM_QuickInit(FTM0_CH4_PD04, kPWM_EdgeAligned, 3000,0);
        FTM_PWM_QuickInit(FTM0_CH5_PD05, kPWM_EdgeAligned, 3000,0);
        FTM_PWM_QuickInit(FTM0_CH6_PD06, kPWM_EdgeAligned, 3000,0);
        FTM_PWM_QuickInit(FTM0_CH7_PD07, kPWM_EdgeAligned, 3000,0);
	/* ���ж� */
	GPIO_QuickInit( HW_GPIOC, 6, kGPIO_Mode_IPU );
	GPIO_CallbackInstall( HW_GPIOC, GPIO_ISR );
	GPIO_ITDMAConfig( HW_GPIOC, 6, kGPIO_IT_FallingEdge, false );   /*�������ش��� */
	/* ���ж� */
	GPIO_QuickInit( HW_GPIOC, 7, kGPIO_Mode_IPD );
	GPIO_CallbackInstall( HW_GPIOC, GPIO_ISR );
	GPIO_ITDMAConfig( HW_GPIOC, 7, kGPIO_IT_RisingEdge, false );    /*�½��ش��� */
	/* PCLK */
	GPIO_QuickInit( HW_GPIOC, 2, kGPIO_Mode_IPU );
	GPIO_ITDMAConfig( HW_GPIOC, 2, kGPIO_DMA_RisingEdge, false );
	/* ����ͷ���ݿڣ����� */
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
        //adc
        ADC_QuickInit(ADC1_SE4B_PC8, kADC_SingleDiff12or13);
        
        
	/* LED */
	GPIO_QuickInit( HW_GPIOC, 4, kGPIO_Mode_OPP );
	GPIO_QuickInit( HW_GPIOC, 5, kGPIO_Mode_OPP );
	GPIO_QuickInit( HW_GPIOB, 17, kGPIO_Mode_OPP );

	/* OLED IO��ʼ�� */
	GPIO_QuickInit( HW_GPIOA, 12, kGPIO_Mode_OPP );
	GPIO_QuickInit( HW_GPIOA, 13, kGPIO_Mode_OPP );
	GPIO_QuickInit( HW_GPIOA, 14, kGPIO_Mode_OPP );
	GPIO_QuickInit( HW_GPIOA, 15, kGPIO_Mode_OPP );
	GPIO_QuickInit( HW_GPIOA, 16, kGPIO_Mode_OPP );
        GPIO_QuickInit( HW_GPIOE, 25, kGPIO_Mode_OPP );

	//UART_QuickInit( UART3_RX_PC16_TX_PC17, 9600 );                /* ���� */
	init_ov7620();                                                  /* ����ͷ��ʼ�� */
	
        OLED_Init();                                                    /* OLED��ʼ�� */

        SYSTICK_DelayInit();
	init_steer();                                                   /* �����ʼ�� */
	GPIO_ITDMAConfig( HW_GPIOC, 6, kGPIO_IT_FallingEdge, true );
	GPIO_ITDMAConfig( HW_GPIOC, 7, kGPIO_IT_RisingEdge, true );
	GPIO_ITDMAConfig( HW_GPIOC, 2, kGPIO_DMA_RisingEdge, true );
        
	/* ���򿪹س�ʼ�� */
	GPIO_QuickInit( HW_GPIOE, 0, kGPIO_Mode_IPU );                  /* �� */
	GPIO_QuickInit( HW_GPIOE, 1, kGPIO_Mode_IPU );                  /*�� */
	GPIO_QuickInit( HW_GPIOE, 3, kGPIO_Mode_IPU );                  /* �� */
	GPIO_QuickInit( HW_GPIOE, 2, kGPIO_Mode_IPU );                  /* �� */
	GPIO_QuickInit( HW_GPIOC, 18, kGPIO_Mode_IPU );                 /*�� */
        
        /*����ʹ��*/
        GPIO_WriteBit(HW_GPIOE, 25, 1); 
}


void init_ov7620( void )
{
	/* DMA��ʼ�� */
	DMA_InitTypeDef DMA_InitStruct1 = { 0 };                        /* �����ʼ���ṹ�� */
	DMA_InitStruct1.chl			= HW_DMA_CH0;
	DMA_InitStruct1.chlTriggerSource	= PORTC_DMAREQ;
	DMA_InitStruct1.triggerSourceMode	= kDMA_TriggerSource_Normal;
	DMA_InitStruct1.minorLoopByteCnt	= 1;
	DMA_InitStruct1.majorLoopCnt		= col_num;              /* ��ѭ���ɼ�������һ����ѭ���ɼ�һ�� */

	DMA_InitStruct1.sAddr		= (uint32_t) &(PTC->PDIR) + 1;  /* dmaԴ��ַ��ptc8~15 */
	DMA_InitStruct1.sLastAddrAdj	= 0;
	DMA_InitStruct1.sAddrOffset	= 0;
	DMA_InitStruct1.sDataWidth	= kDMA_DataWidthBit_8;          /* ���ݿ��� */
	DMA_InitStruct1.sMod		= kDMA_ModuloDisable;

	DMA_InitStruct1.dLastAddrAdj	= 0;
	DMA_InitStruct1.dAddrOffset	= 1;
	DMA_InitStruct1.dDataWidth	= kDMA_DataWidthBit_8;
	DMA_InitStruct1.dMod		= kDMA_ModuloDisable;

	DMA_Init( &DMA_InitStruct1 );
	DMA_DisableRequest( HW_DMA_CH0 ); /* �ȹر�DMA���� */

	/* ��������ͷ�Ĵ��� */
	uint8_t i = 0;


	/*
	 * ��ʼ��SCCB�������ţ�SCCB��һ�ּ򻯵�������I2C��ͨ��Э�顣
	 * ��ʼ��PTC3��PTC0����ΪSCCB�е�SDA��SCL��
	 */
	GPIO_QuickInit( HW_GPIOC, 0, kGPIO_Mode_OPP );
	GPIO_QuickInit( HW_GPIOC, 3, kGPIO_Mode_OPP );
	while ( i == 0 )
		i += LPLD_SCCB_WriteReg( 0x14, 0x24 );  /* QVGA(320*120) */
	while ( i == 1 )
		i += LPLD_SCCB_WriteReg( 0x70, 0xc1 );  /* ������������һ�� */
	while ( i == 2 )
		i += LPLD_SCCB_WriteReg( 0x24, 0x20 );  /* �����ɼ�ģʽ(320*240) */
	while ( i == 3 )
		i += LPLD_SCCB_WriteReg( 0x06, 0xa0 );  /* ���ȿ��� */
}


void init_steer( void )
{
	/*
	 * ռ�ձ� = pwmDuty/10000*100%
	 * ������ֵռ�ձ�=1.5/20=7.5%,ʵ������ݻ�е�ṹ�Ͷ���������е���
	 */
	FTM_PWM_QuickInit(FTM2_CH0_PB18, kPWM_EdgeAligned, 80, 1200);
	/* FTM_PWM_InvertPolarity(HW_FTM2,HW_FTM_CH1,kFTM_PWM_HighTrue); */
}
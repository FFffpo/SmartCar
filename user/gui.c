/**
 ******************************************************************************
 * @file    gui.c
 * @author  袁双杰
 * @version V1.0.0
 * @date    2018.10.11
 * @brief   适用于0.96寸oled液晶屏的菜单，建议在车子运行的时候关闭菜单。
 *          出现闪烁是正常现象，因为设置了一个计数刷新的策略
 *          文末还有flash存取操作的代码。flash存取就是可以让数据改变后，断电也保存下来
 ******************************************************************************
 */
#include "math.h"
#include "chlib_k.h"
#include "varieble.h"
#include "img_processing.h"
#include "oled.h"
#include "init.h"
#include "ftm.h"
#include "adc.h"
#include "gpio.h"
#include "common.h"
#include "uart.h"
#include "gui.h"

int			page		= 1;    /* 初始为第一页 */
int			menuRow		= 1;    /* 记录当前是第几排 值可以为1、2、3、4、5  修改了排数的除外 */
int			keyState	= 0;    /* 用于记录五项开关的值 */
const unsigned char	row_pos[5]	= { 0, 13, 26, 39, 52 };
int			key_counter	= 0;
int			key_temp	= 0;
int			temp1;
int			temp2;
int			temp3;
int			temp4;
int			temp5;
int			temp6;
int			temp7;
int			clearCount = 0;


/*
 *  GPIO_QuickInit(HW_GPIOE, 0, kGPIO_Mode_IPU);   //右
 *  GPIO_QuickInit(HW_GPIOE, 1, kGPIO_Mode_IPU);   //上
 *  GPIO_QuickInit(HW_GPIOE, 3, kGPIO_Mode_IPU);   //左
 *  GPIO_QuickInit(HW_GPIOE, 2, kGPIO_Mode_IPU);   //中
 *  GPIO_QuickInit(HW_GPIOC, 18, kGPIO_Mode_IPU);  //下
 */

int keyCheck( void )                                    /*按键检测 */
{
	int	key_lable = 0;
	int	temp1, temp2;

	/* 得出哪个键按下的标签位 */
	if ( 0 == GPIO_ReadBit( HW_GPIOE, 1 ) )         /* up */
	{
		key_lable = 1;
	}
	if ( 0 == GPIO_ReadBit( HW_GPIOC, 18 ) )        /* down */
	{
		key_lable = 2;
	}
	if ( 0 == GPIO_ReadBit( HW_GPIOE, 3 ) )         /* left */
	{
		key_lable = 3;
	}
	if ( 0 == GPIO_ReadBit( HW_GPIOE, 0 ) )         /* right */
	{
		key_lable = 4;
	}
	if ( 0 == GPIO_ReadBit( HW_GPIOE, 2 ) )         /* select */
	{
		key_lable = 5;
	}


	/* 标签位为0，没有键按下 */
	if ( key_lable == 0 )                           /* 没有按键按下 */
	{
		if ( key_temp != 0 )                    /* 之前有按键按下 */
		{
			temp1		= key_counter;  /*按下的计数器 */
			temp2		= key_temp;     /* 之前的按键的标号 */
			key_counter	= 0;
			key_temp	= 0;
			if ( temp1 > 1 )                /* 消抖 */
			{
				return(temp2);
			}else {
				return(0);
			}
		}else return(0);
	}
	/* 标签位不为0，有键按下 */
	else{                           /*有按键按下 */
		if ( key_temp == 0 )    /* 之前没有检测到按键按下 */
		{
			key_temp = key_lable;
		}
		key_counter++;

		return(0);
	}
}


void menu()
{
	if ( clearCount == 20 )
	{
		clearCount = 0;
		if ( page != 1 )
		{
			OLED_Clear();
		}
	}else
		++clearCount;
	if ( page == 1 )
	{       /* 第一页显示图像 */
		dispimage();
	}else if ( page == 2 )
	{
		MenuShow2();
	}else if ( page == 3 )
	{
		MenuShow3();
	}
	FlashValueOperate();                                    /* 根据按键信息，修改变量值 */
	keyState = keyCheck();
	SignMove();                                             /* 行标移动+页数显示 */
	OLED_ShowNum_12061( 0, row_pos[menuRow - 1], page, 1 ); /* 当前页数和当前行显示 */
	OLED_Refresh_Gram();
}


void SignMove()
{
	if ( page == 1 )
	{
		OLED_ShowString_1206( 13, row_pos[0], "image", 1 );
	}
	if ( 0 )
	{
	}else {
		if ( keyState == KEY_UP )
		{
			if ( menuRow == 1 )
			{
				if ( page == 2 )        /* 第二页第一行按上键 */
				{
					page = 1;
					OLED_Clear();
				}else if ( page == 1 )  /* 第一页翻到最后一页 */
				{
					page = pageMax;
					OLED_Clear();
					menuRow = 5;
				}else {
					page = page - 1;
					OLED_Clear();
					menuRow = 5;
				}
			}else {
				--menuRow;

				OLED_Fill( 0, row_pos[menuRow], 13, row_pos[menuRow] + 13, 0 ); /* 填充之前写的数字 */
			}
		}else if ( keyState == KEY_DOWN )
		{
			if ( page == 1 )                                                        /* 第一页按下键 */
			{
				page	= 2;
				menuRow = 1;
				OLED_Clear();
				OLED_ShowNum_12061( 0, row_pos[menuRow - 1], page, 1 );
			}else if ( menuRow == 5 )                                               /* 最后一行按下键 */
			{
				if ( page == pageMax )
				{
					page	= 1;
					menuRow = 1;
					OLED_Clear();
				}else {
					page	= page + 1;
					menuRow = 1;
					OLED_Clear();
				}
			}else {                                                                         /* 翻页 */
				OLED_Clear();
				OLED_Fill( 0, row_pos[menuRow - 1], 13, row_pos[menuRow - 1] + 13, 0 ); /* 填充之前写的数字 */
				menuRow = menuRow + 1;
			}
		}             else if ( keyState == KEY_MID )
		{
		}else {
		}
	}
}


void FlashValueOperate()
{
	/* 修改每个变量的值 */

	if ( keyState != 0 )
		OLED_Clear();
	if ( page == 2 )                /* 菜单第一页 */
	{
		switch ( menuRow )      /* 判断行数 */
		{                       /* 妹有第零行 */
		case 0: {
		} break;
		/* 从case1开始  第一行 */
		case 1: {
			if ( keyState == KEY_LEFT )
			{
				flashData[0]--;
			}else if ( keyState == KEY_RIGHT )
			{
				flashData[0]++;
			}
		} break;
		/*  第二行 */
		case 2: {
			if ( keyState == KEY_LEFT )
			{
				flashData[1]--;
			}else if ( keyState == KEY_RIGHT )
			{
				flashData[1]++;
			}
		} break;
		case 3: {
			if ( keyState == KEY_LEFT )
			{
			}else if ( keyState == KEY_RIGHT )
			{
			}
		} break;
		case 4: {
			if ( keyState == KEY_LEFT )
			{
			}else if ( keyState == KEY_RIGHT )
			{
			}
		} break;
		}
	}
}


void MenuShow2()                                                                                                        /* 显示不了浮点数，但是类型是浮点数类型，所以现实的时候还是要进行处理 */
{
	OLED_ShowString_1206( 16, row_pos[0], "string1", 1 );
	if ( number < 0 )
	{
		OLED_ShowString_1206( 65, row_pos[0], "-", 1 ); OLED_ShowNum_12061( 72, row_pos[0], flashData[0], 1 );
	}else { OLED_ShowString_1206( 65, row_pos[0], "+", 1 );  OLED_ShowNum_12061( 72, row_pos[0], flashData[0], 1 ); }     /* OLED_ShowString_1206(72,row_pos[0],"value1",1); */

	OLED_ShowString_1206( 16, row_pos[1], "string2", 1 );
	if ( number < 0 )
	{
		OLED_ShowString_1206( 65, row_pos[1], "-", 1 ); OLED_ShowNum_12061( 72, row_pos[1], flashData[1], 1 );
	}else { OLED_ShowString_1206( 65, row_pos[1], "+", 1 );  OLED_ShowNum_12061( 72, row_pos[1], flashData[1], 1 ); }     /* OLED_ShowString_1206(72,row_pos[0],"value1",1); */

	OLED_ShowString_1206( 16, row_pos[2], "string3", 1 );
	if ( number < 0 )
	{
		OLED_ShowString_1206( 65, row_pos[2], "-", 1 ); OLED_ShowNum_12061( 72, row_pos[2], -number, 1 );
	}else { OLED_ShowString_1206( 65, row_pos[2], "+", 1 );  OLED_ShowNum_12061( 72, row_pos[2], number, 1 ); }     /* OLED_ShowString_1206(72,row_pos[0],"value1",1); */

	OLED_ShowString_1206( 16, row_pos[3], "string4", 1 );
	if ( number < 0 )
	{
		OLED_ShowString_1206( 65, row_pos[3], "-", 1 ); OLED_ShowNum_12061( 72, row_pos[3], -number, 1 );
	}else { OLED_ShowString_1206( 65, row_pos[3], "+", 1 );  OLED_ShowNum_12061( 72, row_pos[3], number, 1 ); }     /* OLED_ShowString_1206(72,row_pos[0],"value1",1); */

	OLED_ShowString_1206( 16, row_pos[4], "string5", 1 );
	if ( number < 0 )
	{
		OLED_ShowString_1206( 65, row_pos[4], "-", 1 ); OLED_ShowNum_12061( 72, row_pos[4], -number, 1 );
	}else { OLED_ShowString_1206( 65, row_pos[4], "+", 1 );  OLED_ShowNum_12061( 72, row_pos[4], number, 1 ); }     /* OLED_ShowString_1206(72,row_pos[0],"value1",1); */
}


void MenuShow3()
{
	OLED_ShowString_1206( 16, row_pos[0], "string1", 1 );  OLED_ShowNum_12061( 72, row_pos[0], number, 1 );         /* OLED_ShowString_1206(72,row_pos[0],"value1",1); 显示不了浮点数，但是类型是浮点数类型，所以现实的时候还是要进行处理 */
	OLED_ShowString_1206( 16, row_pos[1], "string2", 1 );  OLED_ShowNum_12061( 72, row_pos[1], number, 1 );         /* OLED_ShowString_1206(72,row_pos[1],"value2",1); */
	OLED_ShowString_1206( 16, row_pos[2], "string3", 1 );  OLED_ShowNum_12061( 72, row_pos[2], number, 1 );         /* OLED_ShowString_1206(72,row_pos[2],"value3",1); */
	OLED_ShowString_1206( 16, row_pos[3], "string4", 1 );  OLED_ShowNum_12061( 72, row_pos[3], number, 1 );         /* OLED_ShowString_1206(72,row_pos[3],"value4",1); */
	OLED_ShowString_1206( 16, row_pos[4], "string5", 1 );  OLED_ShowNum_12061( 72, row_pos[4], number, 1 );         /* OLED_ShowString_1206(72,row_pos[4],"value5",1); */
}


/*
 * 以下为flash操作的说明
 * 首先在init了以后一定要
 *        FLASH_Init();   flash初始化
 *       flashDataRead();
 * flash数据是存在内存的一个扇区当中。原理掠过直接将操作
 * 我们的要存取的数据存在flashData[flashInfoNum]里面。可以把它当作一个中转站。比如A是我们想要进行flash存取的数据每次开机后，
 * flashDataRead()函数执行过后。我们再把flashData[0]赋值给A，这是读取。在写入时，比如A的值在程序运行过程中发生了变化，先把A赋值给flashData[0]，
 * 再执行flashDataSave()函数。那么我们就把我们想要存取的数据写进了对应扇区。每次我们想要存取的数据都必须有一一对应。
 * 对于初次进行flash操作一定要先写入一遍在进行存取，即先不执行flashDataRead();，而直接执行flashDataSave()
 * 之后在每次开机先进行flashDataRead();
 * flash寿命有限，严禁在循环，中断中使用！
 * flash寿命有限，严禁在循环，中断中使用！
 * flash寿命有限，严禁在循环，中断中使用！
 * flash寿命有限，严禁在循环，中断中使用！
 * 一般是按下某个按键，进行写入存储。实现断电存数据
 * 一般现在程序开始main()函数开始时执行flashDataRead()函数读取断电前的数据。再设置一个按钮，当按下按钮时执行flashDataSave()函数
 */


/* flash写入 */
void flashDataSave()
{
	FLASH_EraseSector( 245 );  FLASH_EraseSector( 244 );  FLASH_EraseSector( 243 );                               /* 先擦除扇区 ，一个扇区2kb擦三个够了吧，不够自己再开*/
	FLASH_WriteSector( 245, (uint8_t *)flashData, sizeof(flashData) );      /* 存数据 */
}


/* flash存取 */
void flashDataRead()
{
	Flash_read( 245, 0, sizeof(flashData),  (uint8_t *)flashData );
}



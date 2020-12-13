/**
 ******************************************************************************
 * @author  ZhifeiWang
 * @version V1.0.1
 * @date    2020.12.05
 * @brief   光电组C
 ******************************************************************************
 */
#include "math.h"
#include "chlib_k.h"
#include "varieble.h"
#include "img_processing.h"
#include "init.h"
#include "ftm.h"
#include "adc.h"
#include "gpio.h"
#include "common.h"
#include "uart.h"
#include "gui.h"
#include "flash.h"
#include "upper_monitor.h"
#include "oled.h"

//阈值、路宽、环岛手调、出入库手调
//4出库、3环岛、2入库

//static void PIT0_CallBack(void);
int degree_calculation(int start);
float RoadWidth(void);
int park(void);

//int lastcardegree=0;
int k;
int k_far;
int flag=0,flag1=0,flag2=0;
int first_left=0;
int motor=0;

int roadwidth=110;

float current_roadwidth;

int main()
{
  
  //DisableInterrupts;
  init();
  flag=0;
  flag1=0;
  first_left=0;
  motor=0;
  PBout(9)=0;
  //电机接口
  //4(L) 7(R) R
  //5(L) 6(R) F
  
  /*PIT_QuickInit(HW_PIT_CH0,1000000);
  PIT_CallbackInstall(HW_PIT_CH0, PIT0_CallBack);
  PIT_ITDMAConfig(HW_PIT_CH0, kPIT_IT_TOF,ENABLE);
  EnableInterrupts;*/
  
  //改变舵机占空比，1200是中间位置，1080向右打死，1320向左打死
  FTM_PWM_ChangeDuty(HW_FTM2, HW_FTM_CH0, 1200);

  FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH5, 0);      
  FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH6, 0); 
  while(1)
  {
 
    searchline_OV7620();
    
    
    //dispimage1();//展示处理后的边线，将图像显示在OLED上
    dispimage();//展示二值图像，无边线处理
    
    current_roadwidth=RoadWidth();
    
    if (PBin(21)==1 && park()==1)
    {
      //手动入库
      FTM_PWM_ChangeDuty(HW_FTM2, HW_FTM_CH0, 1200);
      SYSTICK_DelayMs(150);

      FTM_PWM_ChangeDuty(HW_FTM2, HW_FTM_CH0, 1080);
      SYSTICK_DelayMs(800);

      FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH5, 0);      
      FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH6, 0); 
      FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH4, 1500);      
      FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH7, 1500);
      SYSTICK_DelayMs(200);
      FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH4, 0);      
      FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH7, 0);
      FTM_PWM_ChangeDuty(HW_FTM2, HW_FTM_CH0, 1200);
      motor=0;
      PBout(9)=0;
    }
    
    if (PBin(22)==1 && first_left==1 && flag==0 && motor==1)
    {
      
      if(flag1==0 && current_roadwidth >= roadwidth+10)
      {
        PBout(9)=1;
        flag1=1;
      }
      
      
      if (flag1==1 && current_roadwidth <= roadwidth+5)
      {
        flag2=1;
        PBout(9)=0;
      }

      if(flag1==1 && flag2==1 && current_roadwidth >= roadwidth+16)
      {
        PBout(9)=1;
        
        flag=1;
        FTM_PWM_ChangeDuty(HW_FTM2, HW_FTM_CH0, 1200);
        SYSTICK_DelayMs(150);
        FTM_PWM_ChangeDuty(HW_FTM2, HW_FTM_CH0, 1300);
        SYSTICK_DelayMs(2800);
   
      }
    }
    
    if (flag==1)
    {
      FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH5, 1000);      
      FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH6, 1000);
    }
    
    k=degree_calculation(45);
    /*k_far=degree_calculation(30);
    
    ///调速//////////////////////////////
    if (motor==1 && flag==1)
    {
      if (k_far <= 30 && k_far >=-30 && k <= 30 && k >= -30)
      {
        FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH5, 1800);      
        FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH6, 1800); 
      }
      else
      {
        FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH5, 1100);      
        FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH6, 1100); 
      }
    }*/
    //////////////////////////////////////
    
    if (k > 70)
    {
      first_left=1;
    }
 
    FTM_PWM_ChangeDuty(HW_FTM2, HW_FTM_CH0, 1200 + k);
    //////////调车OLED//////////////////////
    OLED_ShowNum_1206(80,20,Midx[20],1);   
    OLED_ShowNum_1206(0,20,k,1);
    OLED_ShowNum_1206(80,40,current_roadwidth,1);  
    ///////////////////////////////////////
    
    OLED_Refresh_Gram();
    
    if(PBin(16)==1)
    {
      motor=1;
      flag=0;
      flag1=0;
      first_left=0;
      FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH5, 1200);  
      FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH6, 1200);
      if (PBin(23)==1)
      {
        //手动出库
        FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH5, 1100);  
        FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH6, 1100);
        
        FTM_PWM_ChangeDuty(HW_FTM2, HW_FTM_CH0, 1200);
        SYSTICK_DelayMs(500);
        FTM_PWM_ChangeDuty(HW_FTM2, HW_FTM_CH0, 1080);
        SYSTICK_DelayMs(1250);
        //FTM_PWM_ChangeDuty(HW_FTM2, HW_FTM_CH0, 1200);
        //SYSTICK_DelayMs(100);
        FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH5, 1200);  
        FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH6, 1200);
      }
      
    }
    if(PBin(10)==1)
    {
      FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH5, 0);      
      FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH6, 0); 
      motor=0;
      PBout(9)=0;
    }
  }

}

/*static void PIT0_CallBack(void) 
{
  
}*/

int degree_calculation(int start)
{
	uint16_t i = 0;
        
  	int cardegree=0;
    //方向控制算法
	int InclineValue=0;                       	//倾斜度
        float ExcursionValue=0;				//偏移量
    //中线倾斜度计算
    for ( i = 1; i <= 15; i++ )
    {
      	if((Midx[start-i] - Midx[start-i-1] < 24) && (Midx[start-i] - Midx[start-i-1] > -24))
        	InclineValue = InclineValue + (Midx[start-i] - Midx[start-i-2]);	//用差分方法求解中线倾斜度
    }
    //偏移量计算
    for ( i = 1; i <= 8; i++ )
    {
        ExcursionValue = ExcursionValue + (Midx[start-i] - car_center);	//用差分法求解总偏移值
    } 
    cardegree = (int)(InclineValue*3.3156 - 0.2388*ExcursionValue);
    if (cardegree>120) cardegree=120;
    if (cardegree<-120) cardegree=-120;
    return cardegree;
}

float RoadWidth(void)
{
  float ml=0,mr=0;
  for (int i=11;i<=13;i++)
  {
    ml+=Lx[i];
    mr+=Rx[i];
  }
  ml/=3;
  mr/=3;
  return (ml-mr);
}

int park(void)
{
  int num=0;
  for (int i=150; i>=3; )
  {
    do
    {
      i--;
    }while(imgadd[5* col_num + i] < whiteRoad );
    do
    {
      i--;
    }while(imgadd[5* col_num + i] > whiteRoad );
    num+=1;
  }
  
  if (num > 8)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}
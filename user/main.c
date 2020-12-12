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

//阈值、路宽、正常过弯道时间、环岛相邻丢线时间差、环岛手调、出入库手调
//4出库、3环岛、2入库

//static void PIT0_CallBack(void);
int degree_calculation(void);
float RoadWidth(void);

int lastcardegree=0;
int k;
float mid_k;
int flag=0,flag1=0,flag2=0;
int t=0,t1=0,t2=0;
int motor=0;
int roadwidth=120;
float current_roadwidth;
int var[4]={1,2,3,4};

int main()
{
  
  //DisableInterrupts;
  init();
  flag=0;
  flag1=0;
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

  
  while(1)
  {
 
    searchline_OV7620();
    
    
    //dispimage1();//展示处理后的边线，将图像显示在OLED上
    dispimage();//展示二值图像，无边线处理
    
    current_roadwidth=RoadWidth();
    
    if (PBin(21)==1 && current_roadwidth <= roadwidth/4 && current_roadwidth >= 5)
    {
      //手动入库
      FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH5, 0);      
      FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH6, 0);
      motor=0;
    }
    
    if (PBin(22)==1 && flag==0 && motor==1)
    {
      
      if(flag1==0 && current_roadwidth >= roadwidth+10)
      {
        //t1=SYSTICK_GetVal();
        PBout(9)=1;
        flag1=1;
      }
      
      //t=SYSTICK_GetVal();
      
      if (flag1==1 && current_roadwidth <= roadwidth)
      {
        flag2=1;
      }

      if(flag1==1 && flag2==1 && current_roadwidth >= roadwidth+20)
      {
        
        //t2=SYSTICK_GetVal();
        PBout(9)=0;
        //if (t2-t1<=2500)
        //{
          flag=1;
          PBin(9)=1;
          FTM_PWM_ChangeDuty(HW_FTM2, HW_FTM_CH0, 1290);
          SYSTICK_DelayMs(3600);
      
       
        //}
      
        /*else
        {
          flag1=1;
          t1=t2;
        }*/
      }
    }
   
    /*if (flag==1 && current_roadwidth >= roadwidth+20)
    {
      SYSTICK_DelayMs(800);
    }*/
    
    k=degree_calculation();
 
    FTM_PWM_ChangeDuty(HW_FTM2, HW_FTM_CH0, 1200 + k);
    //////////调车OLED//////////////////////
    OLED_ShowNum_1206(80,20,Midx[20],1);   
    OLED_ShowNum_1206(0,20,k,1);
    OLED_ShowNum_1206(80,40,current_roadwidth,1);  
    OLED_ShowNum_1206(0,40,t2-t1,1);
    ///////////////////////////////////////
    
    OLED_Refresh_Gram();
    
    //////////调车蓝牙///////////////////////
    var[0]=Midx[20];
    var[1]=k;
    var[2]= (int) current_roadwidth;
    if (t2-t1 < 0)
    {
      var[3]= 50;
    }
    else
    {
      var[3]=t2-t1;
    }
    vcan_sendware((uint8_t *)var,sizeof(var));
    //////////////////////////////////////////
    if(PBin(16)==1)
    {
      motor=1;
      flag=0;
      flag1=0;
      FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH5, 1200);  
      FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH6, 1200);
      if (PBin(23)==1)
      {
        //手动出库
        FTM_PWM_ChangeDuty(HW_FTM2, HW_FTM_CH0, 1200);
        SYSTICK_DelayMs(800);
        FTM_PWM_ChangeDuty(HW_FTM2, HW_FTM_CH0, 1080);
        SYSTICK_DelayMs(1250);
        FTM_PWM_ChangeDuty(HW_FTM2, HW_FTM_CH0, 1200);
        SYSTICK_DelayMs(400);
      }
      
    }
    if(PBin(10)==1)
    {
      FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH5, 0);      
      FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH6, 0); 
      motor=0;
    }
  }

}

/*static void PIT0_CallBack(void) 
{
  
}*/

int degree_calculation(void)
{
	uint16_t i = 0;
        
  	int cardegree=0;
    //方向控制算法
	int InclineValue=0;                       	//倾斜度
        float ExcursionValue=0;				//偏移量
    //中线倾斜度计算
    for ( i = 1; i <= 15; i++ )
    {
      	if((Midx[45-i] - Midx[45-i-1] < 6) && (Midx[45-i] - Midx[45-i-1] > -6))
        	InclineValue = InclineValue + (Midx[45-i] - Midx[45-i-2]);	//用差分方法求解中线倾斜度
    }
    //偏移量计算
    for ( i = 1; i <= 8; i++ )
    {
        ExcursionValue = ExcursionValue + (Midx[45-i] - car_center);	//用差分法求解总偏移值
    } 
     //OLED_ShowNum_1206(80,20,InclineValue,1);
     //OLED_ShowNum_1206(80,40,ExcursionValue,1);
    cardegree = (int)(InclineValue*3.3156 - 0.2388*ExcursionValue);
    if (cardegree>120) cardegree=120;
    if (cardegree<-120) cardegree=-120;
/*    if((cardegree-lastcardegree<50)&&(cardegree-lastcardegree>-50))
    {
      return lastcardegree;
    }
    lastcardegree=cardegree;*/
    return cardegree;
}

float RoadWidth(void)
{
  float ml=0,mr=0;
  for (int i=25;i<=27;i++)
  {
    ml+=Lx[i];
    mr+=Rx[i];
  }
  ml/=3;
  mr/=3;
  return (ml-mr);
}
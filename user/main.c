/**
 ******************************************************************************
 * @author  ZhifeiWang
 * @version V1.0.1
 * @date    2020.12.05
 * @brief   �����C
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

//��ֵ��·�������ֵ���������ֵ�
//4���⡢3������2���

//static void PIT0_CallBack(void);
int degree_calculation(void);
float RoadWidth(void);
int park(void);

//int lastcardegree=0;
int k;
int flag=0,flag1=0,flag2=0;
int first_left=0;
int motor=0;

int roadwidth=120;

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
  //����ӿ�
  //4(L) 7(R) R
  //5(L) 6(R) F
  
  /*PIT_QuickInit(HW_PIT_CH0,1000000);
  PIT_CallbackInstall(HW_PIT_CH0, PIT0_CallBack);
  PIT_ITDMAConfig(HW_PIT_CH0, kPIT_IT_TOF,ENABLE);
  EnableInterrupts;*/
  
  //�ı���ռ�ձȣ�1200���м�λ�ã�1080���Ҵ�����1320�������
  FTM_PWM_ChangeDuty(HW_FTM2, HW_FTM_CH0, 1200);

  
  while(1)
  {
 
    searchline_OV7620();
    
    
    //dispimage1();//չʾ�����ı��ߣ���ͼ����ʾ��OLED��
    dispimage();//չʾ��ֵͼ���ޱ��ߴ���
    
    current_roadwidth=RoadWidth();
    
    if (PBin(21)==1 && park()==1)
    {
      //�ֶ����
      FTM_PWM_ChangeDuty(HW_FTM2, HW_FTM_CH0, 1080);
      SYSTICK_DelayMs(900);
      FTM_PWM_ChangeDuty(HW_FTM2, HW_FTM_CH0, 1200);
      FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH5, 0);      
      FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH6, 0); 
      FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH4, 1200);      
      FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH7, 1200);
      SYSTICK_DelayMs(200);
      FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH4, 0);      
      FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH7, 0);
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

      if(flag1==1 && flag2==1 && current_roadwidth >= roadwidth+15)
      {
        PBout(9)=1;
        
        flag=1;
        FTM_PWM_ChangeDuty(HW_FTM2, HW_FTM_CH0, 1300);
        SYSTICK_DelayMs(3400);
   
      }
    }
    
    k=degree_calculation();
    
    if (k > 70)
    {
      first_left=1;
    }
 
    FTM_PWM_ChangeDuty(HW_FTM2, HW_FTM_CH0, 1200 + k);
    //////////����OLED//////////////////////
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
        //�ֶ�����
        FTM_PWM_ChangeDuty(HW_FTM2, HW_FTM_CH0, 1200);
        SYSTICK_DelayMs(400);
        FTM_PWM_ChangeDuty(HW_FTM2, HW_FTM_CH0, 1080);
        SYSTICK_DelayMs(1100);
        FTM_PWM_ChangeDuty(HW_FTM2, HW_FTM_CH0, 1200);
        SYSTICK_DelayMs(100);
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

int degree_calculation(void)
{
	uint16_t i = 0;
        
  	int cardegree=0;
    //��������㷨
	int InclineValue=0;                       	//��б��
        float ExcursionValue=0;				//ƫ����
    //������б�ȼ���
    for ( i = 1; i <= 15; i++ )
    {
      	if((Midx[50-i] - Midx[50-i-1] < 15) && (Midx[50-i] - Midx[50-i-1] > -15))
        	InclineValue = InclineValue + (Midx[50-i] - Midx[50-i-2]);	//�ò�ַ������������б��
    }
    //ƫ��������
    for ( i = 1; i <= 8; i++ )
    {
        ExcursionValue = ExcursionValue + (Midx[50-i] - car_center);	//�ò�ַ������ƫ��ֵ
    } 
    cardegree = (int)(InclineValue*3.3156 - 0.2388*ExcursionValue);
    if (cardegree>120) cardegree=120;
    if (cardegree<-120) cardegree=-120;
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

int park(void)
{
  int num=0;
  for (int i=150; i>=3; )
  {
    do
    {
      i--;
    }while(imgadd[20* col_num + i] < whiteRoad );
    do
    {
      i--;
    }while(imgadd[20* col_num + i] > whiteRoad );
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
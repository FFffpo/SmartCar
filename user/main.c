/**
 ******************************************************************************
 * @author  ZhifeiWang
 * @version V1.0.0
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

//static void PIT0_CallBack(void);
//float Middle(void);

int lastcardegree=0;
int k;
float mid_k;
int flag=0,flag1=0,flag2=0;
int degree_calculation(void);
int t1=0,t2=0;
int motor=0;

int main()
{
  
  //DisableInterrupts;
  init();
  flag=0;
  flag1=0;
  flag2=0;
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
    
    /*if ((Rx[20]-Lx[20])<=20&&(Rx[20]-Lx[20])>=-20)
    {
      FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH5, 0);      
      FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH6, 0); 
    }*/
    
    k=degree_calculation();
    
    if (flag==0&&motor==1)
    {
      if(flag1==0&&Lx[20]>=140)
      {
        t1=SYSTICK_GetVal();
        PBout(9)=1;
        flag1=1;
      }
      if(flag1==1&&(SYSTICK_GetVal()-t1)>400&&flag2==0&&Lx[20]>=140)
      {
        t2=SYSTICK_GetVal();
        PBout(9)=0;
        flag2=1;
        if (t2-t1<=800)
        {
          flag=1;
          PBin(9)=1;
          FTM_PWM_ChangeDuty(HW_FTM2, HW_FTM_CH0, 1300);
          SYSTICK_DelayMs(2800);
        }
        else
        {
          flag1=flag2=0;
        }
      }
    }
    /*mid_k=Middle();
    
    if (flag==0)      
    {
      if (k<=30&&k>=-30)
      {
        if (mid_k-car_center>=5||mid_k-car_center<=-5)
        {
          flag=1;
          FTM_PWM_ChangeDuty(HW_FTM2, HW_FTM_CH0, 1320);
          SYSTICK_DelayMs(5000);
        }
      }
    }*/

    FTM_PWM_ChangeDuty(HW_FTM2, HW_FTM_CH0, 1200 + k);
    
    OLED_ShowNum_1206(80,20,Midx[20],1);   
    OLED_ShowNum_1206(0,20,k,1);
    OLED_ShowNum_1206(80,40,Lx[20],1);  
    OLED_ShowNum_1206(80,20,t2-t1,1);
    
    OLED_Refresh_Gram();
    
    if(PBin(16)==1)
    {
      motor=1;
      flag=0;
      flag1=0;
      flag2=0;
      FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH5, 1200);  
      FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH6, 1200);
      FTM_PWM_ChangeDuty(HW_FTM2, HW_FTM_CH0, 1100);
      SYSTICK_DelayMs(2000);
      FTM_PWM_ChangeDuty(HW_FTM2, HW_FTM_CH0, 1200);

      
    }
    if(PBin(10)==1)
    {
      FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH5, 0);      
      FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH6, 0); 
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
      	if((Midx[45-i] - Midx[45-i-1] < 6) && (Midx[45-i] - Midx[45-i-1] > -6))
        	InclineValue = InclineValue + (Midx[45-i] - Midx[45-i-2]);	//�ò�ַ������������б��
    }
    //ƫ��������
    for ( i = 1; i <= 8; i++ )
    {
        ExcursionValue = ExcursionValue + (Midx[45-i] - car_center);	//�ò�ַ������ƫ��ֵ
    } 
     //OLED_ShowNum_1206(80,20,InclineValue,1);
     //OLED_ShowNum_1206(80,40,ExcursionValue,1);
    //cardegree = (int)(InclineValue*2.763 - 0.199*ExcursionValue);
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

/*float Middle(void)
{
  double ave=0;
  for (int i=3;i<=8;i++)
  {
    ave+=Midx[i];
  }
  ave/=5;
  return ave;
}*/
/*int square(void)
{
  int l1=Lx[];
}*/
//ע������ͷ���ӽǵȣ���Ҫʱ�ɶ�ɨ���������������
//��ȷ������λ��mid
//�Ȳ����õ�������ƶ�����������������ȶ�ƫ�ƺ���б�ȵ�ֵ��Ȼ��ȡֵ������ù�ϵ
//���������Ϻ󣬰�װ��������ʱ������ο�������ͬ��ͨ����ת��ñ���������ֵ����⼸�飬�õ����Թ�ϵ

/**
 ******************************************************************************
 * @author  ZhifeiWang
 * @version V1.0.1
 * @date    2021.3.13
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
#include "pid.h"

int8_t var[4]={1,2,3,4};

//��ֵ��·�������ֵ���������ֵ�
//4���⡢3������2���

int degree_calculation(int start);
int park(void);
void Speed_Measure(void);

int k,k_far;
int kR1,kR2,kL1,kL2;
int motor=0;
float theta=0;

int16_t RightCadence=0,LeftCadence=0;
uint8_t RightDir=0,LeftDir=0;
PID_type Left_Motor;
PID_type Right_Motor;

int16_t Tarspeed=0;
int dif=0;
int16_t Left_duty=800;
int16_t Right_duty=800;

int main()
{
  
  DisableInterrupts;
  init();
  FTM_QD_ClearCount(HW_FTM1);
  LPTMR_ClearCounter();
  
  PID_Init(&Left_Motor);
  PID_Init(&Right_Motor);
  
  Left_Motor.Kp = 55;
  Left_Motor.Ki = 29;
  Left_Motor.Kd = 10;
  
  Right_Motor.Kp = 43;
  Right_Motor.Ki = 18;
  Right_Motor.Kd = 6;
  motor=0;
  //����ӿ�
  //4(L) 7(R) R
  //5(L) 6(R) F
  
  PIT_QuickInit(HW_PIT_CH0, 50000);		//50msPIT��ʱ�ж� ����
  PIT_CallbackInstall(HW_PIT_CH0, Speed_Measure);
  PIT_ITDMAConfig(HW_PIT_CH0, kPIT_IT_TOF,ENABLE);
  
  
  //�ı���ռ�ձȣ�1200���м�λ�ã�1080���Ҵ�����1320�������
  FTM_PWM_ChangeDuty(HW_FTM2, HW_FTM_CH0, 1200);

  FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH5, 0);      
  FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH6, 0); 
  
  EnableInterrupts;
  
  while(1)
  {
    searchline_OV7620();
    k=degree_calculation(37);
    k_far=degree_calculation(18);
    FTM_PWM_ChangeDuty(HW_FTM2, HW_FTM_CH0, 1200 + k);
   //dispimage1();//չʾ�����ı��ߣ���ͼ����ʾ��OLED��
    dispimage();//չʾ��ֵͼ���ޱ��ߴ���
    if (motor==1)
    {
      if ( k<40 && k>-40 && k_far>-60 && k_far<60 )
      {
        Tarspeed=55;
      }
      else
      {
        Tarspeed=40;
      }
    }
    
    //////////����OLED//////////////////////
    OLED_ShowNum_1206(80,20,Midx[20],1);   
    OLED_ShowNum_1206(0,20,theta,1);
    OLED_ShowNum_1206(0,37,LeftCadence,1);
    OLED_ShowNum_1206(80,37,RightCadence,1);
    OLED_ShowNum_1206(0,48,k,1);
    OLED_ShowNum_1206(80,48,k_far,1);
    OLED_Refresh_Gram();
    //���
    if (PBin(21)==1 && park()==1)
    {
      DisableInterrupts;
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
    }
    /*
    //���뻷��
    if (PBin(22)==1)
    {
      
    }
    */
    /////////////��������//////////////////////
    var[0]=LeftCadence;
    var[1]=RightCadence;
    var[2]=k;
    var[3]=k_far;
    vcan_sendware((int8_t *)var,sizeof(var));
    /////////////////////////////////////////
    
    if(PBin(16)==1)
    {
      motor=1;
      EnableInterrupts;
      Tarspeed=0;
      Left_duty=800;
      Right_duty=800;
      
      //�ֶ�����
      if (PBin(23)==1)
      {
        DisableInterrupts;
        FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH5, 1100);  
        FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH6, 1100);
        
        FTM_PWM_ChangeDuty(HW_FTM2, HW_FTM_CH0, 1200);
        SYSTICK_DelayMs(500);
        FTM_PWM_ChangeDuty(HW_FTM2, HW_FTM_CH0, 1080);
        SYSTICK_DelayMs(1250);
        FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH5, 1200);  
        FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH6, 1200);
        EnableInterrupts;
      }
      
    }
    // ͣ��
    if(PBin(10)==1)
    {     
      Tarspeed=0;     
      motor=0;
    }

  }
}


int degree_calculation (int start)
{
  
  uint16_t i = 0;
  int cardegree=0;
    //��������㷨
  int InclineValue=0;                       	//��б��
  float ExcursionValue=0;				//ƫ����
    //������б�ȼ���
  for ( i = 1; i <= 15; i++ )
    {
      	if((Midx[start-i] - Midx[start-i-1] < 24) && (Midx[start-i] - Midx[start-i-1] > -24))
        	InclineValue = InclineValue + (Midx[start-i] - Midx[start-i-2]);	//�ò�ַ������������б��
    }
    //ƫ��������
    for ( i = 1; i <= 10; i++ )
    {
        ExcursionValue = ExcursionValue + (Midx[start-i] - car_center);	//�ò�ַ������ƫ��ֵ
    } 
    cardegree = (int)(InclineValue*3.3156 - 0.2388*ExcursionValue);
    if (cardegree>120) cardegree=120;
    if (cardegree<-120) cardegree=-120;
    return cardegree;
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

//1����,0����
void Speed_Measure()
{

  FTM_QD_GetData(HW_FTM1, &LeftCadence, &LeftDir);//��ȡ������1�������ֵ   ���ص������Ѿ�������    ������˳ʱ��תʱΪ������ʱ��תʱΪ��
  FTM_QD_ClearCount(HW_FTM1);//��ȡ������1�������ֵ 
  LeftCadence/=75; 
  if (LeftCadence<0)
  {
    LeftCadence=-LeftCadence;
  }

  RightCadence=LPTMR_PC_ReadCounter();//��ȡ������2�������ֵ 
  LPTMR_ClearCounter();//��0������2���������ֵ
  RightCadence/=10; 
    
  if(PCin(4)==1)//������˳ʱ��תʱΪ������ʱ��תʱΪ��
  {
    RightDir=1;
   }
  else
  {
    RightDir=0;
   }
  
  theta=k/120*0.7;
  dif=(int)Tarspeed*0.4*tanf(theta);
  
  Left_Motor.Tarspeed=Tarspeed-dif;
  Right_Motor.Tarspeed=Tarspeed+dif;
      
  Left_Motor.Realspeed=LeftCadence;
  PID_Ctrl(&Left_Motor);
  Left_duty+=Left_Motor.Output;
  if (Left_duty>=3500)Left_duty=3500;
  if (Left_duty<0)Left_duty=0;
  Right_Motor.Realspeed=RightCadence;
  PID_Ctrl(&Right_Motor);
  Right_duty+=Right_Motor.Output;
  if (Right_duty>=3500)Right_duty=3500;
  if (Right_duty<0)Right_duty=0;
  FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH5,Left_duty);
  FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH6,Right_duty);
}


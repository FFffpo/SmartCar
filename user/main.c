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

int lastcardegree=0;
int degree_calculation(void);

int main()
{

  init();
  //����ӿ�
  //4(L) 7(R) R
  //5(L) 6(R) F
  
  
  //�ı���ռ�ձȣ�1200���м�λ�ã�1080���Ҵ�����1320�������
  FTM_PWM_ChangeDuty(HW_FTM2, HW_FTM_CH0, 1200);

  
  while(1)
  {
 
    searchline_OV7620();
    
    
    //dispimage1();//չʾ�����ı��ߣ���ͼ����ʾ��OLED��
    dispimage();//չʾ��ֵͼ���ޱ��ߴ���
    
    int k=degree_calculation();

    FTM_PWM_ChangeDuty(HW_FTM2, HW_FTM_CH0, 1200 + k);
    
    OLED_ShowNum_1206(80,20,Midx[40],1);
    
    //OLED_ShowNum_1206(20,20,k,1);
    OLED_Refresh_Gram();
    
    if(PBin(16)==1)
    {
      FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH5, 1200);  
      FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH6, 1200);
    }
    if(PBin(10)==1)
    {
      FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH5, 0);      
      FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH6, 0); 
    }
  }

}

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
//ע������ͷ���ӽǵȣ���Ҫʱ�ɶ�ɨ���������������
//��ȷ������λ��mid
//�Ȳ����õ�������ƶ�����������������ȶ�ƫ�ƺ���б�ȵ�ֵ��Ȼ��ȡֵ������ù�ϵ
//���������Ϻ󣬰�װ��������ʱ������ο�������ͬ��ͨ����ת��ñ���������ֵ����⼸�飬�õ����Թ�ϵ

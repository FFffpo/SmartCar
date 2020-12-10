/**
 ******************************************************************************
 * @author  ZhifeiWang
 * @version V1.0.0
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

int lastcardegree=0;
int degree_calculation(void);

int main()
{

  init();
  //电机接口
  //4(L) 7(R) R
  //5(L) 6(R) F
  
  
  //改变舵机占空比，1200是中间位置，1080向右打死，1320向左打死
  FTM_PWM_ChangeDuty(HW_FTM2, HW_FTM_CH0, 1200);

  
  while(1)
  {
 
    searchline_OV7620();
    
    
    //dispimage1();//展示处理后的边线，将图像显示在OLED上
    dispimage();//展示二值图像，无边线处理
    
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
//注意摄像头的视角等，必要时可对扫描的行数进行设置
//先确定中线位置mid
//先不启用电机，逐步推动车调整电机参数，先读偏移和倾斜度的值，然后取值代入求得关系
//舵机调试完毕后，安装编码器，时间间隔与参考代码相同，通过空转获得编码器的数值，多测几组，得到线性关系

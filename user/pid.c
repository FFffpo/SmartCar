#include "pid.h"

void PID_Init(PID_type* Motor)
{
  Motor->Kp = 0;
  Motor->Ki = 0;
  Motor->Kd = 0;
 
  Motor->Output = 0;
  Motor->Realspeed = 0;
  Motor->Tarspeed = 0;
  
  Motor->Cur_Error=0;
  Motor->Pre_Error=0;
  Motor->Prepre_Error=0;
}

void PID_Ctrl(PID_type* Motor)
{
  
  Motor->Prepre_Error= Motor->Pre_Error;
  Motor->Pre_Error = Motor->Cur_Error;
  Motor->Cur_Error = Motor->Tarspeed - Motor->Realspeed;
  Motor->Output = (int)(Motor->Kp*(Motor->Cur_Error- Motor->Pre_Error)+ Motor->Ki*(Motor->Cur_Error)+Motor->Kd*(Motor->Cur_Error- 2*Motor->Pre_Error + Motor->Prepre_Error));
  
}
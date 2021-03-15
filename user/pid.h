#ifndef PID_H_
#define PID_H_

#include "chlib_k.h"

typedef struct                                   //PID调节结构体
{
  float Kp, Ki, Kd;
  float Cur_Error, Pre_Error, Prepre_Error;
  int16_t Realspeed;
  int16_t Tarspeed;
  int16_t Output;
}PID_type;


void PID_Init(PID_type* Motor);

void PID_Ctrl(PID_type* Motor);
#endif /* PID_H_ */
/*
 * CompFilter.h
 *  Created on: 04.04.2014
 *      Author: PK
 */

#include <math.h>
#include "stm32f4xx.h"


#define K 0.02

double GetPitchAngle(double PitchAngle, double Gx_Cur,double Gx_Prev,uint64_t x_Cur,uint64_t x_Prev,s16 AccX,s16 AccY,s16 AccZ);
double GetYawAngle(double yawAngle, double Gy_Cur,double Gy_Prev,uint64_t y_Cur,uint64_t y_Prev,s16 AccX,s16 AccY,s16 AccZ);

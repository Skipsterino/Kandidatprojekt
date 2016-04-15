/*
 * invers_kinematik.h
 *
 * Created: 4/8/2016 3:12:02 PM
 *  Author: chrma018
 */ 

#ifndef INVERS_KINEMATIK_H_
#define INVERS_KINEMATIK_H_

#include <math.h>

#include "structs.h"

triple_uint Kar_To_Pos(float x, float y, float z);
triple_float Pos_To_Kar(float p1, float p2, float p3);

triple_float Kar_To_Cyl_Leg1(float x, float y, float z);
triple_float Kar_To_Cyl_Leg2(float x, float y, float z);
triple_float Kar_To_Cyl_Leg3(float x, float y, float z);
triple_float Kar_To_Cyl_Leg4(float x, float y, float z);
triple_float Kar_To_Cyl_Leg5(float x, float y, float z);
triple_float Kar_To_Cyl_Leg6(float x, float y, float z);

#endif /* INVERS_KINEMATIK_H_ */
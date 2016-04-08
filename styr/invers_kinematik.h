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

six_uchar Kar_To_Pos(triple_float kar);
triple_float Pos_To_Kar(six_uchar pos);

#endif /* INVERS_KINEMATIK_H_ */
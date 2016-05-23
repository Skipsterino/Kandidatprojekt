/*
 *        File: invers_kinematik.c
 *     Version: 1.0
 * Last edited: 20/5/2016 
 *      Author: chrma018
 */ 

#include "invers_kinematik.h"

//Skalfaktor för omvandling mellan vinkel och servoposition
#define ANG_TO_POS_SCALE 195.37860814f
#define PI 3.14159265359f
//Bensegmentslängder och vanligt använda storheter i cm
#define L1 5.1f
#define L2 6.6f
#define L2_SQ 43.56f
#define L3_SQ 176.89f //L3 = 13.3
#define L3_L2_2 175.56f

 
triple_uint Kar_To_Pos(float x, float y, float z)
{	
	//Hjälpvariabler
	float r_leg = sqrt(pow(x,2)+pow(y,2));
	float r_outer_leg = r_leg - L1;
	float r_outer_leg_sq = pow(r_outer_leg,2);
	float z_sq= pow(z,2);
	float p_outer_sq =  r_outer_leg_sq + z_sq;
	
	//Invers kinematik
	float v1 = asin((y)/r_leg);
	float v2 = atan(-z/r_outer_leg)-acos((L3_SQ-L2_SQ-p_outer_sq)/(-2*L2*sqrt(p_outer_sq)));
	float v3 = acos((p_outer_sq - L3_SQ - L2_SQ)/L3_L2_2);

	//Gör om vinklarna till postionförskjutningar och sparar som unsigned int
	unsigned int pos1 = (unsigned int)(ANG_TO_POS_SCALE*v1);
	unsigned int pos2 = (unsigned int)(ANG_TO_POS_SCALE*v2);
	unsigned int pos3 = (unsigned int)(ANG_TO_POS_SCALE*v3);
	
	//För att skicka ut 3 st int
	triple_uint pos = create_triple_uint(pos1, pos2, pos3);
	return pos; 
}


//Varje ben har en egen funktion pga olika positionering på skelettet

triple_float Kar_To_Cyl_Leg1(float x, float y, float z)
{
	float r = sqrt(pow(x+6, 2) +pow(y+12, 2)); // x offset = 6, y offset=12
	float th = atan((x+6)/(y+12));
	
	//För att skicka ut 3 st floats 
	triple_float triple = create_triple_float(r,th,z);
	return triple;
}

triple_float Kar_To_Cyl_Leg2(float x, float y, float z)
{
	float r = sqrt(pow(x+6, 2) +pow(y+12, 2)); // x offset = 6, y offset=12
	float th = atan(-(x+6)/(y+12));
	
	//För att skicka ut 3 st floats 
	triple_float triple = create_triple_float(r,th,z);
	return triple;
}

triple_float Kar_To_Cyl_Leg3(float x, float y, float z)
{
	float r = sqrt(pow(x + 10, 2) +pow(y, 2)); // x offset = 10, y offset=0
	float th = PI/2 - atan(y/(x + 10));
	
	//För att skicka ut 3 st floats 
	triple_float triple = create_triple_float(r,th,z);
	return triple;
}

triple_float Kar_To_Cyl_Leg4(float x, float y, float z)
{
	float r = sqrt(pow(x+10, 2) + pow(y, 2)); // x offset = 10, y offset=0
	float th= -PI/2+atan(y/(x+10));
	
	//För att skicka ut 3 st floats 
	triple_float triple = create_triple_float(r,th,z);
	return triple;
}

triple_float Kar_To_Cyl_Leg5(float x, float y, float z)
{
	float r = sqrt(pow(x+6, 2) +pow(12-y, 2)); // x offset = 6, y offset=12
	float th = PI - atan((x+6)/(12-y));
	
	//För att skicka ut 3 st floats 
	triple_float triple = create_triple_float(r,th,z);
	return triple;
}

triple_float Kar_To_Cyl_Leg6(float x, float y, float z)
{
	float r = sqrt(pow(x+6, 2) +pow(12-y, 2)); // x offset = 6, y offset=12
	float th = PI + atan((x+6)/(12-y));
	
	//För att skicka ut 3 st floats 
	triple_float triple = create_triple_float(r,th,z);
	return triple;
}


/*
 * invers_kinematik.c
 *
 * Created: 4/8/2016 3:13:10 PM
 *  Author: chrma018
 */ 

#include "invers_kinematik.h"

float v1;
float v2;
float v3;

float pi=3.14159265359;

//Räknar ut kartesiska koordinater (kar) mha servoposition (pos). Returnerar kar som en triple_float.
//DENNA TAR EJ HÄNSYN TILL OFFSETS!!! EV INTE RELEVANT FUNKTION?
//triple_float Pos_To_Kar(float p1, float p2, float p3)
//{
	////Skalfaktor mellan pos och vinkel (som mäts i RADIANER)
	//float pos_to_ang_scale = 0.0051182676011; 
	//
	////Benlängder i cm
	//float L1 = 5.1;
	//float L2 = 6.6;
	//float L3 = 13.3;
//
	//v1= pos_to_ang_scale * (float)(p1);
	//v2= pos_to_ang_scale * (float)(p2);
	//v3= pos_to_ang_scale * (float)(p3);
	//
	////Kinematik 
	//float x = L1*sin(v1) + L2*sin(v1)*cos(v2) + L3*cos(v1)*cos(v2 + v3);
	//float y = L1*cos(v1) + L2*cos(v1)*cos(v2) + L3*cos(v1)*cos(v2 + v3);
	//float z = -L2*sin(v2) - L3*sin(v2 + v3);
	//
	//triple_float kar = create_triple_float(x,y,z);
	//return kar;
//}

//Räknar ut servopositioner (pos) mha kartesiska koordinater (kar). Returnerar positionerna som en triple_uint. OBS! Konvertering till uchar sker i annan fil, t ex main.
triple_uint Kar_To_Pos(float x, float y, float z)
{
	//Skalfaktor för omvandling mellan vinkel och servoposition
	float ang_to_pos_scale = 195.37860814;
	float pi = 3.1415926536;
	
	//Benlängder i cm
	float L1 = 5.1; 
	float L2 = 6.6;
	float L2_sq = 43.56;
	float L3 = 13.3;
	float L3_sq = 176.89;
	float L3_L2_2 = 175.56;
	
	//Hjälpvariabler
	float r_leg = sqrt(pow(x,2)+pow(y,2));
	float r_outer_leg = r_leg - L1;
	float r_outer_leg_sq = pow(r_outer_leg,2);
	float z_sq= pow(z,2);
	float p_outer_sq =  r_outer_leg_sq + z_sq;
	
	//Invers kinematik
	v1 = asin((y)/r_leg);
	v2 = atan(-z/r_outer_leg)-acos((L3_sq-L2_sq-p_outer_sq)/(-2*L2*sqrt(p_outer_sq)));
	v3 = acos((p_outer_sq - L3_sq - L2_sq)/L3_L2_2);

	//Gör om vinklarna till postionförskjutningar och sparar som unsigned int
	unsigned int pos1 = (unsigned int)(ang_to_pos_scale*v1);
	unsigned int pos2 = (unsigned int)(ang_to_pos_scale*v2);
	unsigned int pos3 = (unsigned int)(ang_to_pos_scale*v3);
	
	triple_uint pos = create_triple_uint(pos1, pos2, pos3);
	return pos; //måste göra subtraktion av offset innan konvertering till char, så skicka ut unsigned int, formatera om sen.
}

triple_float Kar_To_Cyl_Leg1(float x, float y, float z)
{
	float r = sqrt(pow(x+6, 2) +pow(y+12, 2));
	float th = atan((x+6)/(y+12));
	
	triple_float triple = create_triple_float(r,th,z);
	return triple;
}

triple_float Kar_To_Cyl_Leg2(float x, float y, float z)
{
	float r = sqrt(pow(x+6, 2) +pow(y+12, 2));
	float th = atan(-(x+6)/(y+12));
	
	triple_float triple = create_triple_float(r,th,z);
	return triple;
}

triple_float Kar_To_Cyl_Leg3(float x, float y, float z)
{
	float r = sqrt(pow(x + 10, 2) +pow(y, 2));
	float th = pi/2 - atan(y/(x + 10));
	
	triple_float triple = create_triple_float(r,th,z);
	return triple;
}

triple_float Kar_To_Cyl_Leg4(float x, float y, float z)
{
	float r = sqrt(pow(x+10, 2) + pow(y, 2));
	float th= -pi/2+atan(y/(x+10));
	triple_float triple = create_triple_float(r,th,z);
	return triple;
}

triple_float Kar_To_Cyl_Leg5(float x, float y, float z)
{
	float r = sqrt(pow(x+6, 2) +pow(12-y, 2));
	float th = pi - atan((x+6)/(12-y));
	
	triple_float triple = create_triple_float(r,th,z);
	return triple;
}

triple_float Kar_To_Cyl_Leg6(float x, float y, float z)
{
	float r = sqrt(pow(x+6, 2) +pow(12-y, 2));
	float th = pi + atan((x+6)/(12-y));
	
	triple_float triple = create_triple_float(r,th,z);
	return triple;
}


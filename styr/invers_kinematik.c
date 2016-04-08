/*
 * invers_kinematik.c
 *
 * Created: 4/8/2016 3:13:10 PM
 *  Author: chrma018
 */ 

#include "invers_kinematik.h"

//Räknar ut kartesiska koordinater (kar) mha servoposition (pos). Returnerar kar som en triple_float.
triple_float Pos_To_Kar(six_uchar pos)
{
	float pos_to_ang_scale = 0.0051182676011; //Skalfaktor mellan pos och vinkel (som mäts i RADIANER)
	unsigned int first_servo_offset = 0x01; //Lika med 0?
	unsigned int second_servo_offset = 0x01; //Lika med 0?
	unsigned int third_servo_offset = 0x01; //Skild från 0
	float leg_offset_x = 15;
	float leg_offset_y = 10;
	float leg_offset_z = 0;
	
	float L1 = 5; //Benlängder i cm
	float L2 = 7;
	float L3 = 12;
	
	//Plocka fram LS resp. MS Byte av servopositionerna
	unsigned char p1LS = pos.a;
	unsigned char p1MS = pos.b;
	unsigned char p2LS = pos.c;
	unsigned char p2MS = pos.d;
	unsigned char p3LS = pos.e;
	unsigned char p3MS = pos.f;
	
	//Kombinera dessa till unsigned ints ty passar bättre för nedanstående beräkningar
	unsigned int p1 = p1MS; //OBS, här hamnar MS på LS-platserna i p1, varför nedanstående skiftning görs så MS hamnar rätt i p1
	p1 = (p1<<8);
	p1 |= (unsigned int)p1LS; //Adderar till LS på LS-platserna i p1
	unsigned int p2 = p2MS;
	p2 = (p2<<8);
	p2 |= (unsigned int)p2LS;
	unsigned int p3 = p3MS;
	p3 = (p3<<8);
	p3 |= (unsigned int)p3LS;
	
	float v1= pos_to_ang_scale * (float)(p1 - 0x1FF + first_servo_offset);
	float v2= pos_to_ang_scale * (float)(p2 - 0x1FF + second_servo_offset);
	float v3= pos_to_ang_scale * (float)(p3 - 0x1FF + third_servo_offset);
	
	float x = L1*sin(v1) + L2*sin(v1)*cos(v2) + L3*cos(v1)*cos(v2 + v3) + leg_offset_x;
	float y = L1*cos(v1) + L2*cos(v1)*cos(v2) + L3*cos(v1)*cos(v2 + v3) + leg_offset_y;
	float z = -L2*sin(v2) - L3*sin(v2 + v3) + leg_offset_z;
	triple_float kar = create_triple_float(x,y,z);
	return kar;
}

//Räknar ut servopositioner (pos) mha kartesiska koordinater (kar). Returnerar positionerna som en six_uchar. LS Byte först för varje servoposition! (pos1LS, pos1MS,...)
six_uchar Kar_To_Pos(triple_float kar)
{
	float ang_to_pos_scale = 195.37860814;
	float pi = 3.1415926536;
	
	unsigned int first_servo_offset = 0x01; //Lika med 0?
	unsigned int second_servo_offset = 0x01; //Lika med 0?
	unsigned int third_servo_offset = 0x01; //Skild från 0
	
	float L1 = 5; //Benlängder i cm
	float L2 = 7;
	float L2_sq = 49;
	float L3 = 12;
	float L3_sq = 144;
	
	float leg_offset_x = 15;
	float leg_offset_y = 10;
	
	float x = kar.a;
	float y = kar.b;
	float z = kar.c;
	
	//Hjälpvariabler
	float r_leg = sqrt(pow((x-leg_offset_x),2)+pow((y-leg_offset_y),2));
	float r_outer_leg = r_leg - L1;
	float r_outer_leg_sq = pow(r_outer_leg,2);
	float z_sq= pow(z,2);
	float big_ass_thing = asin((L3_sq-r_outer_leg_sq-z_sq-L2_sq)/(-2*L2)/sqrt(r_outer_leg_sq+z_sq));

	//Invers kinematik
	float v1 = asin((y-leg_offset_y)/r_leg);
	float v2 = big_ass_thing + atan(r_outer_leg/z);
	float v3 = pi -big_ass_thing + asin((L2_sq-r_outer_leg_sq-z_sq-L3_sq)/(-2*L3)/sqrt(r_outer_leg_sq+z_sq));

	//Spara först positionerna som unsigned int (beräkningarna passar bättre med att hantera både LS och MS byte samtidigt som en unsigned int)
	unsigned int pos1 =(unsigned int)(ang_to_pos_scale*v1) + 0x1FF + first_servo_offset;
	unsigned int pos2 =(unsigned int)(ang_to_pos_scale*v2) + 0x1FF + second_servo_offset;
	unsigned int pos3 =(unsigned int)(ang_to_pos_scale*v3) + 0x1FF + third_servo_offset;
	
	//Dela upp positionen i LS respektive MS Byte, spara i unsigned chars (unsigned chars passar bättre vid kontakt med servona)
	unsigned char pos1LS = pos1; //Plockar ut LS Byte av positionen för det första servot (översta servot)
	unsigned char pos1MS = (pos1>>8); //Som ovan men med MS Byte
	unsigned char pos2LS = pos2;
	unsigned char pos2MS = (pos2>>8);
	unsigned char pos3LS = pos3;
	unsigned char pos3MS = (pos3>>8);
	
	six_uchar pos = create_six_uchar(pos1LS, pos1MS, pos2LS, pos2MS, pos3LS, pos3MS);
	return pos;
}
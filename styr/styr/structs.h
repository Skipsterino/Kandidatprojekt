/*
 * structs.h
 *
 * Created: 4/8/2016 2:50:26 PM
 *  Author: chrma018
 */ 


#ifndef STRUCTS_H_
#define STRUCTS_H_

//structs som används i koden
typedef struct{
	float a;
	float b;
	float c;
} triple_float;

typedef struct{
	unsigned int a;
	unsigned int b;
	unsigned int c;
} triple_uint;

typedef struct{
	float a;
	float b;
} double_float;


typedef struct{
	unsigned char a;
	unsigned char b;
	unsigned char c;
	unsigned char d;
	unsigned char e;
	unsigned char f;
} six_uchar;

typedef struct{
	unsigned char a;
	unsigned char b;
} double_uchar;

triple_float create_triple_float(float a, float b, float c);

triple_uint create_triple_uint(unsigned int a, unsigned int b, unsigned int c);

six_uchar create_six_uchar(unsigned char a, unsigned char b, unsigned char c, unsigned char d, unsigned char e, unsigned char f);

double_uchar create_double_uchar(unsigned char a, unsigned char b);

double_float create_double_float(float a, float b);

#endif /* STRUCTS_H_ */
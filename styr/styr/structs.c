#include "structs.h"

//Returnerar en triple_float bestående av tre givna floats.
triple_float create_triple_float(float a, float b, float c)
{
	triple_float this;
	this.a = a;
	this.b = b;
	this.c = c;
	return this;
}

//Returnerar en double_float bestående av tre givna floats.
double_float create_double_float(float a, float b)
{
	double_float this;
	this.a = a;
	this.b = b;
	return this;
}

//Returnerar en triple_uint bestående av tre givna unsigned ints. OBS! En unsigned int är 2 byte lång till skillnad mot en unsigned char, som är 1 byte lång!
triple_uint create_triple_uint(unsigned int a, unsigned int b, unsigned int c)
{
	triple_uint this;
	this.a = a;
	this.b = b;
	this.c = c;
	return this;
}

//Returnerar en six_uchar bestående av sex givna unsigned chars.
six_uchar create_six_uchar(unsigned char a, unsigned char b, unsigned char c, unsigned char d, unsigned char e, unsigned char f)
{
	six_uchar this;
	this.a = a;
	this.b = b;
	this.c = c;
	this.d = d;
	this.e = e;
	this.f = f;
	return this;
}

//Returnerar en double_uchar bestående av två givna unsigned chars.
double_uchar create_double_uchar(unsigned char a, unsigned char b)
{
	double_uchar this;
	this.a = a;
	this.b = b;
	return this;
}
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
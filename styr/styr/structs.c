#include "structs.h"

triple_float create_triple_float(float a, float b, float c)
{
	triple_float this;
	this.a = a;
	this.b = b;
	this.c = c;
	return this;
}

double_float create_double_float(float a, float b)
{
	double_float this;
	this.a = a;
	this.b = b;
	return this;
}

triple_uint create_triple_uint(unsigned int a, unsigned int b, unsigned int c)
{
	triple_uint this;
	this.a = a;
	this.b = b;
	this.c = c;
	return this;
}

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

double_uchar create_double_uchar(unsigned char a, unsigned char b)
{
	double_uchar this;
	this.a = a;
	this.b = b;
	return this;
}
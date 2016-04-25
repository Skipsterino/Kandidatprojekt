/**
* @file structs.h
* @author Erik
* @date 25 apr 2016
* @brief Useful structs
*
* Contains some structs that are useful for returning groups of values from functions.
*/

#ifndef STRUCTS_H_
#define STRUCTS_H_

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

/**
* @brief Creates a triple_float
*
* @param a Float #1
* @param b Float #2
* @param c Float #3
*/
triple_float create_triple_float(float a, float b, float c);

/**
* @brief Creates a triple_uint
*
* @param a Unsigned int #1
* @param b Unsigned int #2
* @param c Unsigned int #3
*/
triple_uint create_triple_uint(unsigned int a, unsigned int b, unsigned int c);

/**
* @brief Creates a six_uchar
*
* @param a Unsigned char #1
* @param b Unsigned char #2
* @param c Unsigned char #3
* @param d Unsigned char #4
* @param e Unsigned char #5
* @param f Unsigned char #6
*/
six_uchar create_six_uchar(unsigned char a, unsigned char b, unsigned char c, unsigned char d, unsigned char e, unsigned char f);

/**
* @brief Creates a double_uchar
*
* @param a Unsigned char #1
* @param b Unsigned char #2
*/
double_uchar create_double_uchar(unsigned char a, unsigned char b);

/**
* @brief Creates a double_float
*
* @param a Float #1
* @param b Float #2
*/
double_float create_double_float(float a, float b);

#endif /* STRUCTS_H_ */
/**
* @file invers_kinematik.h
* @author chrma018
* @date 17/5/2016 
* @version 1.0
* @brief Functions for inverse kinematics
*
* Are used for calculating servo angles from positions in different coordinate systems.    
*/


#ifndef INVERS_KINEMATIK_H_
#define INVERS_KINEMATIK_H_

#include <math.h>

#include "structs.h"

/**
* @brief Cartesian coordinates to servo position
*
* Calculates servo position without consideration to offset, 
* given coordinates in Cartesian coordinates.
*
* @param x The length in x-direction in cm
* @param y The length in y-direction in cm
* @param z The length in z-direction in cm
*
* @return The servopositions as triple_uint, a struct containing three unsigned integers
*/
triple_uint Kar_To_Pos(float x, float y, float z);

/**
* @brief Cartesian koodinates to cylindrical coordinates, for leg1
*
* Calculates cylindrical coordinates,
* given Cartesian koodinates.

* @param p1 The iner servo position 
* @param p2 The middle servo position
* @param p3 The outer servo position
*
* @return The Cartesian koodinates as triple_float, a struct containing three unsigned floats.
*/
triple_float Kar_To_Cyl_Leg1(float x, float y, float z);

/**
* @brief Cartesian koodinates to cylindrical coordinates, for leg2
*
* Calculates cylindrical coordinates,
* given Cartesian koodinates.
*
* @param x The length in x-direction in cm
* @param y The length in y-direction in cm
* @param z The length in z-direction in cm
*
* @return The servopositions as triple_float, a struct containing three unsigned floats.
*/
triple_float Kar_To_Cyl_Leg2(float x, float y, float z);

/**
* @brief Cartesian koodinates to cylindrical coordinates, for leg3
*
* Calculates cylindrical coordinates,
* given Cartesian koodinates.
*
* @param x The length in x-direction in cm
* @param y The length in y-direction in cm
* @param z The length in z-direction in cm
*
* @return The servopositions as triple_float, a struct containing three unsigned floats.
*/
triple_float Kar_To_Cyl_Leg3(float x, float y, float z);

/**
* @brief Cartesian koodinates to cylindrical coordinates, for leg4
*
* Calculates cylindrical coordinates,
* given Cartesian koodinates.
*
* @param x The length in x-direction in cm
* @param y The length in y-direction in cm
* @param z The length in z-direction in cm
*
* @return The servopositions as triple_float, a struct containing three unsigned floats.
*/
triple_float Kar_To_Cyl_Leg4(float x, float y, float z);

/**
* @brief Cartesian koodinates to cylindrical coordinates, for leg5
*
* Calculates cylindrical coordinates,
* given Cartesian koodinates.
*
* @param x The length in x-direction in cm
* @param y The length in y-direction in cm
* @param z The length in z-direction in cm
*
* @return The servopositions as triple_float, a struct containing three unsigned floats.
*/
triple_float Kar_To_Cyl_Leg5(float x, float y, float z);

/**
* @brief Cartesian koodinates to cylindrical coordinates, for leg6
*
* Calculates cylindrical coordinates,
* given Cartesian koodinates.
*
* @param x The length in x-direction in cm
* @param y The length in y-direction in cm
* @param z The length in z-direction in cm
*
* @return The servopositions as triple_float, a struct containing three unsigned floats.
*/
triple_float Kar_To_Cyl_Leg6(float x, float y, float z);

#endif /* INVERS_KINEMATIK_H_ */
#ifndef _MATHUTIL_H_
#define _MATHUTIL_H_

/*MathUtil.h - Declarations for miscellaneous math utilities*/

#include <math.h>

//some definition about Pi.
const float kPi = 3.14159265f;
const float k2Pi = kPi * 2.0f;
const float kPiOver2 = kPi / 2.0f;
const float k1OverPi = 1.0f / kPi;
const float k1Over2Pi = 1.0f / k2Pi;

//Limiting the angle between -Pi to Pi.

extern float wrapPi(float theta);

//Safe anti-trigonometric-funciton
extern float safeAcos(float x);

//Calculate the sin and the cos
inline void sinCos(float* returnSin, float* returnCos, float theta)
{
	*returnSin = sin(theta);
	*returnCos = sin(theta);
}

#endif // !_MATHUTIL_H_


// preprocessor directives
#ifndef UTILS_H
#define UTILS_H
#include <iostream>
#include <stdlib.h>
using namespace std;

// debug definitions
//#define              DEBUG         1

// math pi definition
#define              PI            3.1415926535897932384626433832795

// global constants
static const float TWO_PI = 2.0f * PI;
static const float PI_OVER_180 = PI / 180.0f;


inline float scaleDegrees(float theta) {
	if (theta > 0.0f)
		while ((theta >= 360.0f) || (theta > 180.0f))
			theta -= 360.0f;
	else if (theta < 0.0f)
		while ((theta <= -360.0f) || (theta < -180.0f))
			theta += 360.0f;
	return theta;
} // scaleDegrees(GLfloat)


inline float scaleRadians(float theta) {
	if (theta > 0.0f)
		while ((theta >= TWO_PI) || (theta > PI))
			theta -= TWO_PI;
	else if (theta < 0.0f)
		while ((theta <= -TWO_PI) || (theta < -PI))
			theta += TWO_PI;
	return theta;
} // scaleRadians(GLfloat)


inline float degreesToRadians(float theta) {
	return scaleDegrees(theta) * PI_OVER_180;
} // degreesToRadians(GLfloat)


inline float radiansToDegrees(float theta) {
	return scaleRadians(theta) / PI_OVER_180;
} // radiansToDegrees(GLfloat)


inline float frand(const float min = 0.0f, const float max = 1.0f) {
	return min + (max - min) * (float)rand() / (float)(RAND_MAX + (float)1);
} // frand()


inline int irand(const int min = 0, const int max = 1) {
	return min + rand() % max;
} // irand()


inline float randSign() {
	return (rand() % 2) ? -1.0f : 1.0f;
} // randSign()



inline float sign(const float f) {
	return (f < 0.0f) ? -1.0f : 1.0f;
} // sign(const GLfloat)
#endif

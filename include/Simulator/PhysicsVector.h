
// Description:     This class describes a 3-dimensional vector.

#ifndef VECTOR_H
#define VECTOR_H
#include <cmath>
#include <iostream>
#include "Simulator/utils.h"
using namespace std;


static const float DEFAULT_VECTOR_TRANSLATE[3] = {0.0f, 0.0f, 0.0f};
static const float DEFAULT_VECTOR_ROTATE[3]    = {0.0f, 0.0f, 0.0f};
static const float DEFAULT_VECTOR_SCALE[3]     = {1.0f, 1.0f, 1.0f};

// describes a 3-dimensional vector
class PhysicsVector
{
    public:
        float x, y, z;
        float translate[3], rotate[3], scale[3];
        
        PhysicsVector(const float dx = 0.0f, const float dy = 0.0f, const float dz = 0.0f);
        PhysicsVector(const PhysicsVector &v);

        ~PhysicsVector();

        bool set(const float dx = 0.0f,	const float dy = 0.0f, const float dz = 0.0f);
        bool set(const PhysicsVector &v);
        void translated(const float dx, const float dy, const float dz);
        void translated(const PhysicsVector &v);
        void rotated(float theta);
        void rotateRelative(float theta);
        void scaled(float s);


        bool setPolar(float magnitude = 1.0f, float theta = 0.0f, float dz = 0.0f);
        bool setDifference(const PhysicsVector &dest, const PhysicsVector &source = PhysicsVector());
        bool setAngle(const float theta = 0.0f);
        bool setMagnitude(const float mag = 1.0f);
        //bool setNorm(const float mag = 1.0f);
        bool setPerpendicular();
        bool setAverage(const PhysicsVector v[], const int n = 1);
        bool normalize();


        float angle() const;
        float magnitude() const;
        //float norm()  const;
        PhysicsVector perp();
        float perpendicularDotProduct(const PhysicsVector &v) const;

        // <  overloaded operators>
        PhysicsVector& operator  =(const PhysicsVector &v);
        PhysicsVector  operator  +(const PhysicsVector &v);
        PhysicsVector  operator  -(const PhysicsVector &v);
        PhysicsVector& operator +=(const PhysicsVector &v);
        PhysicsVector& operator -=(const PhysicsVector &v);
        PhysicsVector& operator *=(const float scalar);
        bool operator ==(const PhysicsVector &v);
        bool operator !=(const PhysicsVector &v);

        // <friend functions>
        friend bool operator ==(const PhysicsVector &v, const PhysicsVector &v1);
        friend ostream& operator << (ostream &out, const PhysicsVector &v);
        friend PhysicsVector operator -(const PhysicsVector &v);
        friend PhysicsVector operator *(const float scalar, const PhysicsVector &v);
        friend PhysicsVector operator *(const PhysicsVector &v, const float scalar);
        friend PhysicsVector unit(const PhysicsVector &v);
        friend PhysicsVector crossProduct(const PhysicsVector &v1, const PhysicsVector &v2);
        friend float dotProduct(const PhysicsVector &v1, const PhysicsVector &v2);
        friend float angle(const PhysicsVector &v);
        friend float angle(const PhysicsVector &v1, const PhysicsVector &v2);

    protected:
          bool init(const float dx = 0.0f, const float dy = 0.0f, const float dz = 0.0f);
};
#endif

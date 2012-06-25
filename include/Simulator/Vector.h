//
// Filename:        "Vector.h"
//
// Programmer:      Ross Mead
// Last modified:   30Nov2009
//
// Description:     This class describes a 3-dimensional vector.
//

// preprocessor directives
#ifndef VECTOR_H
#define VECTOR_H
#include <cmath>
#include <iostream>
using namespace std;



// global constants
static const float DEFAULT_VECTOR_TRANSLATE[3] = {0.0f, 0.0f, 0.0f};
static const float DEFAULT_VECTOR_ROTATE[3]    = {0.0f, 0.0f, 0.0f};
static const float DEFAULT_VECTOR_SCALE[3]     = {1.0f, 1.0f, 1.0f};
static const bool    DEFAULT_VECTOR_SHOW_LINE    = true;
static const bool    DEFAULT_VECTOR_SHOW_HEAD    = true;
static const float VECTOR_LINE_WIDTH           = 1.0f;
static const float VECTOR_HEAD_HEIGHT          = 0.015f;
static const float VECTOR_HEAD_WIDTH           = 0.5f * VECTOR_HEAD_HEIGHT;



// describes a 3-dimensional vector
class Vector
{
    public:

        // <public data members>
        float x,        y,            z;
        float translate[3], rotate[3], scale[3];
        bool    showLine, showHead;
        
        // <constructors>
        Vector(const float dx         = 0.0f,
               const float dy         = 0.0f,
               const float dz         = 0.0f);
        Vector(const Vector &v);

        // <destructors>
        virtual ~Vector();

        // <virtual public mutator functions>
        virtual bool set(const float dx = 0.0f,
                         const float dy = 0.0f,
                         const float dz = 0.0f);
        virtual bool set(const Vector &v);
        virtual void translated(const float dx,
                                const float dy,
                                const float dz);
        virtual void translated(const Vector &v);
        virtual void rotated(float theta);
        virtual void rotateRelative(float theta);
        virtual void scaled(float s);

        // <public mutator functions>
        bool setPolar(float magnitude = 1.0f,
                      float theta     = 0.0f,
                      float dz        = 0.0f);
        bool setDiff(const Vector &dest, const Vector &source = Vector());
        bool setAngle(const float theta = 0.0f);
        bool setMagnitude(const float mag = 1.0f);
        //bool setNorm(const float mag = 1.0f);
        bool setPerp();
        bool setAvg(const Vector v[], const int n = 1);
        bool normalize();

           // <public utility functions>
        float angle() const;
        float magnitude() const;
        //float norm()  const;
        Vector  perp();
        float perpDot(const Vector &v) const;

        // <virtual overloaded operators>
        virtual Vector& operator  =(const Vector &v);
        virtual Vector  operator  +(const Vector &v);
        virtual Vector  operator  -(const Vector &v);
        virtual Vector& operator +=(const Vector &v);
        virtual Vector& operator -=(const Vector &v);
        virtual Vector& operator *=(const float scalar);
        virtual bool    operator ==(const Vector &v);
        virtual bool    operator !=(const Vector &v);

        // <friend functions>
        friend ostream& operator << (ostream &out, const Vector &v);
        friend Vector   operator -(const Vector &v);
        friend Vector   operator *(const float scalar, const Vector &v);
        friend Vector   operator *(const Vector &v, const float scalar);
        friend Vector   unit(const Vector &v);
        friend Vector   crossProduct(const Vector &v1, const Vector &v2);
        friend float  dotProduct(const Vector &v1,
                                   const Vector &v2);
        friend float  angle(const Vector &v);
        friend float  angle(const Vector &v1, const Vector &v2);

    protected:

        // <virtual protected utility functions>
        virtual bool init(const float dx         = 0.0f,
                          const float dy         = 0.0f,
                          const float dz         = 0.0f);
};  // Vector

#endif

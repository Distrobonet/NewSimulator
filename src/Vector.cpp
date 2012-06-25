//
// Filename:        "Vector.cpp"
//
// Description:     This class implements a 3-dimensional vector.
//

// preprocessor directives
#include <Simulator/Vector.h>

// Default constructor that initializes the vector to the parameterized values.
Vector::Vector(const float dx, const float dy, const float dz)
{
    init(dx, dy, dz);
}   // Vector(const float, const float, const float, const Color)


// Copy constructor that copies the contents of
// the parameterized vector into this vector.
Vector::Vector(const Vector &v)
{
    init(v.x, v.y, v.z);
    *this = v;

    //setColor(v.color);
    for (int i = 0; i < 3; ++i)
    {
        translate[i] = v.translate[i];
        rotate[i]    = v.rotate[i];
        scale[i]     = v.scale[i];
    }
    showLine         = v.showLine;
    showHead         = v.showHead;

}   // Vector(const Vector &)


Vector::~Vector(){}   // ~Vector()



// Attempts to set the xyz-coordinates to the corresponding
// parameterized values, returning true if successful, false otherwise.
bool Vector::set(const float dx, const float dy, const float dz)
{
    x = dx;
    y = dy;
    z = dz;
    return true;
}   // set(const float, const float, const float)




// Attempts to set the xyz-coordinates to based upon the
// parameterized vector, returning true if successful, false otherwise.
bool Vector::set(const Vector &v)
{
    return set(v.x, v.y, v.z);
}   // set(const Vector &v)


// Translates the vector display based on the
// parameterized xyz-coordinate translations.
void Vector::translated(const float dx, const float dy, const float dz)
{
    translate[0] = dx;
    translate[1] = dy;
    translate[2] = dz;
}   // translated(const float, const float, const float)


// Translates the vector display based on the
// parameterized translation vector.
void Vector::translated(const Vector &v)
{
    translated(v.x, v.y, v.z);
}   // translated(const Vector &)



// Rotates the vector display about itself (in 2-dimensions)
// based on the parameterized rotation angle.
void Vector::rotated(float theta)
{
    rotate[2] = theta;
}   // rotated(float)


// Rotates the vector about itself (in 2-dimensions)
// based on the parameterized rotation angle.
void Vector::rotateRelative(float theta)
{
    theta = degreesToRadians(theta);
    set(x * cos(theta) - y * sin(theta), x * sin(theta) + y * cos(theta), z);
}   // rotateRelative(float)


// Scales the vector display based on the parameterized scalar.
void Vector::scaled(const float s)
{
    scale[0] = scale[1] = scale[2] = s;
}   // scaled(const float)


// Attempts to set the vector based on the parameterized polar coordinates,
// returning true if successful, false otherwise.
bool Vector::setPolar(float mag, float theta, float dz)
{
    theta = degreesToRadians(theta);
    return set(mag * cos(theta), mag * sin(theta), dz);
}   // setPolar(float, float, float)




// Attempts to set the vector to the difference from the
// parameterized source to the parameterized destination,
// returning true if successful, false otherwise.
bool Vector::setDiff(const Vector &dest, const Vector &source)
{
    return set(source.x - dest.x, source.y - dest.y, source.z - dest.z);
}   // setDiff(const Vector &, const Vector &)




// Attempts to set the angle to the parameterized angle,
// returning true if successful, false otherwise.
bool Vector::setAngle(const float theta)
{
    return setPolar(magnitude(), theta);
}   // setAngleconst float)



// Attempts to set the magnitude (normal) to the parameterized magnitude,
// returning true if successful, false otherwise.
bool Vector::setMagnitude(const float mag)
{
    if (!normalize()) return false;
    return set(x * mag, y * mag, z * mag);
}   // setMagnitude(const float)


// Attempts to set this vector to its perpendicular vector,
// returning true if successful, false otherwise.
bool Vector::setPerp()
{
    float temp = x;
    x            = -y;
    y            = temp;
    return true;
}   // setPerp()



// Attempts to set the vector based on the average
// of the parameterized vectors, returning true
// if successful, false otherwise.
bool Vector::setAvg(const Vector v[], const int n)
{
    if (n <= 0) return false;
    for (int i = 0; i < n; i++) *this += v[i];
    x /= (float)n;
    y /= (float)n;
    z /= (float)n;
    return true;
}   // avg(const Vector [], const int)



// Attempts to adjust the vector to unit length, returning true if successful, false otherwise.
bool Vector::normalize()
{
    float mag = magnitude();

    // does nothing to zero vectors
    if (mag == 0.0f) return false;
    return set(x / mag, y / mag, z / mag);
}   // normalize()



// Returns the angle of this vector.
float Vector::angle() const
{
    if ((x == 0.0f) && (y == 0.0f) && (z == 0.0f)) return 0.0f;
    return sign(y) *
           radiansToDegrees(acos(dotProduct(unit(*this), unit(Vector(1.0f)))));
}   // angle()



// Returns the magnitude (normal) of this vector.
float Vector::magnitude() const
{
    return sqrt(x * x + y * y + z * z);
}   // magnitude()



// Returns the perpendicular vector of this vector.
Vector Vector::perp()
{
    Vector v = *this;
    v.setPerp();
    return v;
}   // perp()


// Returns the dot product of the perpendicular vector of this vector and the parameterized vector.
float Vector::perpDot(const Vector &v) const
{
    return x * v.x - y * v.y;
}   // perpDot(const Vector &)


// Copies the contents of the parameterized vector into this vector.
Vector& Vector::operator =(const Vector &v)
{
    set(v.x, v.y, v.z);
    //setColor(v.color);
    for (int i = 0; i < 3; ++i)
    {
        translate[i] = v.translate[i];
        rotate[i]    = v.rotate[i];
        scale[i]     = v.scale[i];
    }
    showLine         = v.showLine;
    showHead         = v.showHead;
    return *this;
}   // =(const Vector &)



// Calculates the sum of the contents of the parameterized vector and this vector.
Vector Vector::operator +(const Vector &v)
{
    return Vector(this->x + v.x, this->y + v.y, this->z + v.z);
}   // +(const Vector &)


// Calculates the difference of the contents of the parameterized vector and this vector.
Vector Vector::operator -(const Vector &v)
{
    return *this + -v;
}   // -(const Vector &)



// Adds the contents of the parameterized vector to this vector.
Vector& Vector::operator +=(const Vector &v)
{
    return *this = *this + v;
}   // +=(const Vector &)



// Subtracts the contents of the parameterized vector from this vector.
Vector& Vector::operator -=(const Vector &v)
{
    return *this = *this - v;
}   // +=(const Vector &)


// Multiplies this vector by the parameterized scalar.
Vector& Vector::operator *=(const float scalar)
{
    return *this = *this * scalar;
}   // *=(const Vector &)



// Compares this vector to the parameterized vector, returning true if they are the same, false otherwise.
bool Vector::operator ==(const Vector &v)
{
    return (x == v.x) && (y == v.y) && (z == v.z);
}   // ==(const Vector &)


// Compares this vector to the parameterized vector, returning true if they are not the same, false otherwise.
bool Vector::operator !=(const Vector &v)
{
    return !(*this == v);
}   // ==(const Vector &)


// Outputs the parameterized vector to the parameterized output file stream.
ostream& operator << (ostream &out, const Vector &v)
{
    out << "<" << v.x << ", " << v.y << ", " << v.z << ">";
    return out;
}   // <<(ostream &, const Vector &)


// Returns the negation of the parameterized vector.
Vector operator -(const Vector &v)
{
    return -1.0f * v;
}   // -(const Vector &)


// Returns the parameterized vector multiplied by the parameterized scalar.
Vector operator *(const float scalar, const Vector &v)
{
    return Vector(scalar * v.x, scalar * v.y, scalar * v.z);
}   // *(const float, const Vector &)


// Returns the parameterized vector multiplied by the parameterized scalar.
Vector operator *(const Vector &v, const float scalar)
{
    return scalar * v;
}   // *(const Vector &, const float)


// Returns the unit vector of the parameterized vector.
Vector unit(const Vector &v)
{
    Vector temp = v;
    temp.normalize();
    return temp;
}   // unit(const Vector &)


// Returns the cross product of the parameterized vectors.
Vector crossProduct(const Vector &v1, const Vector &v2)
{
    return Vector(v1.y * v2.z - v2.y * v1.z,
                  v1.z * v2.x - v2.z * v1.x,
                  v1.x * v2.y - v2.x * v1.y);
}   // crossProduct(const Vector &, const Vector &)


// Returns the dot product of the parameterized vectors.
float dotProduct(const Vector &v1, const Vector &v2)
{
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}   // dotProduct(const Vector &, const Vector &)



// Returns the angle of the parameterized vector.
float angle(const Vector &v)
{
    return v.angle();
}   // angle(const Vector &)



// Returns the angle between the parameterized vectors.
float angle(const Vector &v1, const Vector &v2)
{
    return scaleDegrees(v1.angle() - v2.angle());
}   // angle(const Vector &, const Vector &)




// Initializes the vector to the parameterized values, returning true if successful, false otherwise.
bool Vector::init(const float dx, const float dy, const float dz)
{
    set(dx, dy, dz);
    for (int i = 0; i < 3; ++i)
    {
        translate[i] = DEFAULT_VECTOR_TRANSLATE[i];
        rotate[i]    = DEFAULT_VECTOR_ROTATE[i];
        scale[i]     = DEFAULT_VECTOR_SCALE[i];
    }
    showLine         = DEFAULT_VECTOR_SHOW_LINE;
    showHead         = DEFAULT_VECTOR_SHOW_HEAD;
    return true;
}   // init(const float, const float, const float, const Color)

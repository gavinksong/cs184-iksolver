#ifndef VEC3_H
#define VEC3_H

#include <iostream>

/* 
 *  Supported binary operations:
 *    Addition (+)
 *    Subtraction (-)
 *    Negation (-)
 *    Dot product (*)
 *    Scalar Multiplication (*)
 *    Scalar Division (/)
 */

class Vec3 {
  public:
    float x, y, z;
    Vec3 (void) {};
    Vec3 (float x, float y, float z) : x(x), y(y), z(z) {};
    void assign (float x, float y, float z);
    Vec3 reflect (const Vec3&) const;
    Vec3 operator + (const Vec3&) const;
    Vec3 operator - (const Vec3&) const;
    Vec3 operator - () const;
    float operator * (const Vec3&) const;
    Vec3 operator * (float) const;
    Vec3 operator / (float) const;
    void operator += (const Vec3&);
    void operator -= (const Vec3&);
    void operator *= (float);
    void operator /= (float);
};

Vec3 operator * (float, const Vec3&);
Vec3 normalize (const Vec3&);
Vec3 cross (const Vec3&, const Vec3&);
Vec3 hadamard (const Vec3&, const Vec3&);
float length (const Vec3&);

std::ostream& operator << (std::ostream&, const Vec3&);

#endif

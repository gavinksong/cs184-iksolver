#include <cmath>
#include "vec3.h"

void Vec3::assign (float x, float y, float z) {
  this->x = x;
  this->y = y;
  this->z = z;
};

Vec3 Vec3::reflect (const Vec3& v) const {
  Vec3 n = normalize (*this);
  return -v + 2 * (v * n) * n;  
};

Vec3 Vec3::operator + (const Vec3& param) const {
  Vec3 temp;
  temp.x = this->x + param.x;
  temp.y = this->y + param.y;
  temp.z = this->z + param.z;
  return temp;
};

Vec3 Vec3::operator - (const Vec3& param) const {
  Vec3 temp;
  temp.x = this->x - param.x;
  temp.y = this->y - param.y;
  temp.z = this->z - param.z;
  return temp;
};

Vec3 Vec3::operator - (void) const {
  Vec3 temp;
  temp.x = -this->x;
  temp.y = -this->y;
  temp.z = -this->z;
  return temp;
};

float Vec3::operator * (const Vec3& param) const {
  float temp = 0;
  temp += this->x * param.x;
  temp += this->y * param.y;
  temp += this->z * param.z;
  return temp;
};

Vec3 Vec3::operator * (float c) const {
  Vec3 temp;
  temp.x = this->x * c;
  temp.y = this->y * c;
  temp.z = this->z * c;
  return temp;
};

Vec3 Vec3::operator / (float c) const {
  Vec3 temp;
  temp.x = this->x / c;
  temp.y = this->y / c;
  temp.z = this->z / c;
  return temp;
};

Vec3 operator * (float c, const Vec3& v) {
  return v * c;
};

void Vec3::operator += (const Vec3& param) {
  *this = *this + param;
};

void Vec3::operator -= (const Vec3& param) {
  *this = *this - param;
};

void Vec3::operator *= (float c) {
  *this = *this * c;
};

void Vec3::operator /= (float c) {
  *this = *this / c;
};

float length (const Vec3& v) {
  return sqrt (v * v);
};

Vec3 normalize (const Vec3& v) {
  return v / sqrt (v * v);
};

Vec3 cross (const Vec3& lhs, const Vec3& rhs) {
  Vec3 temp;
  temp.x = lhs.y * rhs.z - lhs.z * rhs.y;
  temp.y = lhs.z * rhs.x - lhs.x * rhs.z;
  temp.z = lhs.x * rhs.y - lhs.y * rhs.x;
  return temp;
};

Vec3 hadamard (const Vec3& lhs, const Vec3& rhs) {
  Vec3 temp;
  temp.x = lhs.x * rhs.x;
  temp.y = lhs.y * rhs.y;
  temp.z = lhs.z * rhs.z;
  return temp;
};

std::ostream& operator << (std::ostream& out, const Vec3& v) {
  out << "(" << v.x << ", " << v.y << ", " << v.z << ")";
  return out;
};

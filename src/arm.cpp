#include "arm.h"
#include <cmath>

using namespace Eigen;
using namespace std;

Matrix3f crossmat (const Vector3f& v);
Matrix3f rodriguez (const Vector3f& v);

Arm::Arm (float x, float y, float z) {
  this->tip = Vector3f (x, y, z);
};

Arm::~Arm (void) {
  // Delete all joints.
  for (list<Vector3f*>::iterator it = this->joints.begin ();
       it != this->joints.end ();
       it++) {
    delete *it;
  }
}

void Arm::addJoint (float x, float y, float z) {
  Vector3f *newJoint = new Vector3f (this->tip);
  this->tip = Vector3f (x, y, z);
  this->joints.push_back (newJoint);
};

Matrix<float, 3, Dynamic> Arm::jacobian (void) {
  // Initialize Jacobian.
  Matrix<float, 3, Dynamic> jacobian;
  // For each joint (other than the end effector),
  for (list<Vector3f*>::iterator it = this->joints.begin ();
       it != this->joints.end ();
       it++) {
    // Calculate the diff between joint and end effector.
    Vector3f diff = this->tip - **it;
    // Take the crossmat of the diff and append it to Jacobian.
    jacobian << crossmat (diff);
  }
  return jacobian;
};

void Arm::applyRotations (Vector3f *expmaps) {
  // Set R as identity 3x3 matrix.
  Matrix3f R = Matrix3f::Identity ();
  int i = 0;
  // For each joint (in outward order, other than end effector)
  for (list<Vector3f*>::iterator it = this->joints.begin ();
       it != this->joints.end ();
       it++) {
    // and corresponding expmap,
    Vector3f expmap = expmaps[i++];
    // Apply R to joint.
    **it = R * (**it);
    // Apply rodriguez (expmap) to R.
    R = rodriguez (expmap) * R;
  }
};

void Arm::stepTowards (Vector3f goal) {
  // Take the jacobian.
  Matrix<float, 3, Dynamic> jacobian = this->jacobian ();
  // Calculate error.
  Vector3f err = goal - this->tip;
  // Solve error = jacobian * x.
  Vector3f x = jacobian.colPivHouseholderQr ().solve (err);
  // Turn x into array of Vector3fs.
  int length = this->joints.size () - 1;
  Vector3f *expmaps = new Vector3f[length];
  for (int i = 0; i < length; i++) {
    expmaps[i] = x.block<3, 1> (3*i+1, 1);
  }
  // applyRotations.
  applyRotations (expmaps);
  delete [] expmaps;
};

Matrix3f crossmat (const Vector3f& v) {
  Matrix3f m;
  m << 0, -v(2), v(1),
       v(2), 0, -v(0),
       -v(1), v(0), 0;
  return m;
};

Matrix3f rodriguez (const Vector3f& r) {
  float theta = r.dot(r);
  Vector3f rn = r / theta;
  Matrix3f cross = crossmat (rn);
  return rn * rn.transpose()
         + sin (theta) * cross
         - cos (theta) * cross * cross;
};

#include "arm.h"
#include <cmath>

using namespace Eigen;
using namespace std;

Matrix3f crossmat (const Vector3f& v);
Matrix4f rodriguez (const Vector3f& v);
Matrix4f translation (const Vector3f& v);
Vector3f applyTransform (const Matrix4f& t, const Vector3f& v3);

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

int Arm::numJoints (void) {
  return this->joints.size () + 1;
}

// TODO: Design a better interface. This is lazy.
Matrix3Xf Arm::getJoints (void) {
  Matrix3Xf joints (3, this->numJoints ());
  int i = 0;
  // For each joint (in outward order),
  for (list<Vector3f*>::iterator it = this->joints.begin ();
       it != this->joints.end ();
       it++) {
    joints.block<3,1>(0,i++) = **it;
  }
  joints.block<3,1>(0,i) = this->tip;
  return joints;
};

void Arm::addJoint (float x, float y, float z) {
  Vector3f *newJoint = new Vector3f (this->tip);
  this->tip = Vector3f (x, y, z);
  this->joints.push_back (newJoint);
};

Matrix<float, 3, Dynamic> Arm::jacobian (void) {
  // Initialize Jacobian.
  Matrix3Xf jacobian (3, 3 * this->joints.size ());
  int i = 0;
  // For each joint (in outward order),
  for (list<Vector3f*>::iterator it = this->joints.begin ();
       it != this->joints.end ();
       it++) {
    // Calculate the diff between joint and end effector.
    Vector3f diff = **it - this->tip;
    // Take the crossmat of the diff and append it to Jacobian.
    jacobian.block<3,3>(0,3*i++) = crossmat (diff);
  }
  return jacobian;
};

void Arm::applyRotations (Vector3f *expmaps) {
  // Set transform as identity 4x4 matrix.
  Matrix4f transform = Matrix4f::Identity ();
  int i = 0;
  // For each joint (in outward order),
  for (list<Vector3f*>::iterator it = this->joints.begin ();
       it != this->joints.end ();
       it++) {
    // Take the joint...
    Vector3f joint = **it;
    // Apply the accumulated transform to the joint.
    **it = applyTransform (transform, joint);
    // Add joint rotation to transform.
    transform *= translation (joint) * rodriguez (expmaps[i++])
      * translation (-joint);
  }
  // Apply the final transform to the end effector.
  this->tip = applyTransform (transform, this->tip);
};

void Arm::stepTowards (Vector3f goal) {
  // Take the jacobian.
  Matrix3Xf jacobian = this->jacobian ();
  // Calculate error.
  Vector3f err = goal - this->tip;
  // Solve least-squares for error = jacobian * x.
  VectorXf x = jacobian.jacobiSvd(ComputeThinU|ComputeThinV).solve (err);
  // Turn x into array of Vector3fs.
  int length = this->joints.size ();
  Vector3f *expmaps = new Vector3f[length];
  for (int i = 0; i < length; i++) {
    expmaps[i] = x.block<3,1>(3*i,0);
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

Matrix4f rodriguez (const Vector3f& r) {
  Matrix4f m4 = Matrix4f::Identity ();
  float norm = sqrt (r.dot (r));
  Vector3f rn = r / norm;
  Matrix3f cross = crossmat (rn);
  m4.block<3,3>(0,0) = rn * rn.transpose ()
                       + sin (norm/100) * cross
                       - cos (norm/100) * cross * cross;
  return m4;
};

Matrix4f translation (const Vector3f& v) {
  Matrix4f m = Matrix4f::Identity ();
  m.block<3,1>(0,3) = v;
  return m;
};

Vector3f applyTransform (const Matrix4f& t, const Vector3f& v3) {
  Vector4f v4;
  v4 << v3, 1;
  v4 = t * v4;
  return v4.head (3) / v4(3);
};

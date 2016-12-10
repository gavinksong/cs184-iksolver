#ifndef ARM_H
#define ARM_H

#include "Eigen/Dense"
#include <list>

class Arm {
  private:
    Eigen::Vector3f tip;
    std::list<Eigen::Vector3f*> joints;
    Eigen::Matrix<float, 3, Eigen::Dynamic> jacobian (void);
  public:
    Arm (void) : Arm (0, 0, 0) {};
    Arm (float x, float y, float z);
    ~Arm (void);
    void addJoint (float x, float y, float z);
    void applyRotations (Eigen::Vector3f *expmaps);
    void stepTowards (Eigen::Vector3f goal);
};

#endif

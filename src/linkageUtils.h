#ifndef LINKAGE_UTIL_H
#define LINKAGE_UTIL_H

#include <vector>
#include <Eigen/Dense>
#include "CGL/vector3D.h"

// gives position of new point
Eigen::Vector3f intersectPointDistances(const Eigen::Vector3f& p, float r1, float r2, const Eigen::Vector3f& up);

// gives dp/dq
Eigen::Matrix<float, 3, -1> intersectPointVelocities(const Eigen::Vector3f& p, const Eigen::Matrix<float, 3, -1>& dp, const Eigen::Vector3f& s);

// gives d/dt dp/dq
Eigen::Matrix<float, 3, -1> intersectPointAccelerations(const Eigen::Vector3f& p, const Eigen::Vector3f& v, const Eigen::Matrix<float, 3, -1>& dp, const Eigen::Matrix<float, 3, -1>& ddtdp, const Eigen::Vector3f& s);

Eigen::Matrix3f skewMat(const Eigen::Vector3f& v);

Eigen::Matrix3f rotationMat(const Eigen::Vector3f& r);
Eigen::Matrix3f rotationMat(const Eigen::Matrix3f& skew, float theta);

Eigen::Matrix4f transformationMat(const Eigen::Vector3f& r, const Eigen::Vector3f& t);

std::vector<CGL::Vector3D> convertVectors(const Eigen::Matrix<float, -1, 1>& pos);
CGL::Vector3D convertVector(const Eigen::Vector3f& pos);

Eigen::Matrix<float, -1, 1> convertVectors(const std::vector<CGL::Vector3D>& pos);
Eigen::Vector3f convertVector(const CGL::Vector3D& pos);
#endif
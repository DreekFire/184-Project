#include <iostream>

#include "linkageUtils.h"

// returns positions p, dp/dq, d/dt dp/dq
Eigen::Vector3f intersectPointDistances(const Eigen::Vector3f& p, float r1, float r2, const Eigen::Vector3f& axis) {
  // x^2 - (d-x)^2 = r1^2 - r2^2
  // 2x = r1^2 - r2^2 + d^2
  float r = p.norm();
  float a = (r1 * r1 - r2 * r2 + r * r) / (2 * r);
  // float a = (p.norm2() - (r1 * r1 - r2 * r2)) / (2 * p.norm());
  float b = sqrtf(r1 * r1 - a * a);
  Eigen::Vector3f pUnit = p / r;
  Eigen::Vector3f normal = axis.cross(pUnit);
  Eigen::Vector3f s = a * pUnit + b * normal;
  return s;
}

Eigen::Matrix<float, 3, -1> intersectPointVelocities(const Eigen::Vector3f& p, const Eigen::Matrix<float, 3, -1>& dp, const Eigen::Vector3f& s) {
  Eigen::Matrix<float, 3, -1> vels;
  vels.setZero(3, dp.cols());
  Eigen::Vector3f pRNorm = p / p.squaredNorm();
  // ds/dq = ds/dp * dp/dq
  for (int i = 0; i < dp.cols(); i++) {
    vels.col(i) = pRNorm.cross(dp.col(i)).cross(s);
  }

  if (vels.maxCoeff() > 100 || vels.minCoeff() < -100) {
    std::cout << "uh oh" << std::endl;
  }

  return vels;
}

Eigen::Matrix<float, 3, -1> intersectPointAccelerations(const Eigen::Vector3f& p, const Eigen::Vector3f& v, const Eigen::Matrix<float, 3, -1>& dp, const Eigen::Matrix<float, 3, -1>& ddtdp, const Eigen::Vector3f& s) {
  Eigen::Matrix<float, 3, -1> accels;
  accels.setZero(3, ddtdp.cols());
  Eigen::Vector3f w = s.cross(v) / s.squaredNorm();
  Eigen::Matrix<float, 3, -1> cAccel;
  cAccel.resize(3, dp.cols());
  Eigen::Vector3f pRNorm = p / p.squaredNorm();
  for (int i = 0; i < dp.cols(); i++) {
    // product rule works with cross product too
    // d/dt = (pRNorm x dp_i) x s = (d/dt pRNorm x dp_i) x s + (pRNorm x d/dt dp_i) x s + (pRNorm x dp_i) x d/dt s
    // d/dt pRNorm is parallel to dp_i because they both travel along a circle
    // = qAccel + pRNorm x dp_i x vR
    // cAccel.col(i) = pRNorm.cross(dp.col(i)).cross(v);
    cAccel.col(i) = w.cross(dp.col(i));
  }
  // std::cout << cAccel.transpose() * dp << std::endl;
  // return cAccel;

  return intersectPointVelocities(p, ddtdp, s) + cAccel;

  /*d / dt dp / dq = d / dt(dp / dp2 dp2 / dq) = (d / dt dp / dp2) dp2 / dq + dp / dp2(d / dt dp2 / dq)
  2xn = 2x2 2xn + 2x2 2xn
  d/dt dp/dp2 = d^2 p/dp2^2 dp2/dt (= cAccel)
  rhs = dp/dp2 (d/dt dp2/dq) (calculated for p2 already)*/
    
}

Eigen::Matrix3f skewMat(const Eigen::Vector3f& v) {
  Eigen::Matrix3f A;
  A << 0, -v(2, 0), v(1, 0),
    v(2, 0), 0, -v(0, 0),
    -v(1, 0), v(0, 0), 0;
  return A;
}

Eigen::Matrix3f rotationMat(const Eigen::Vector3f& r) {
  float theta = r.norm();
  if (abs(theta) == 0) {
    return Eigen::Matrix3f::Identity();
  }
  Eigen::Matrix3f K = skewMat(r / theta);
  return rotationMat(K, theta);
}

Eigen::Matrix3f rotationMat(const Eigen::Matrix3f& skew, float theta) {
  return Eigen::Matrix3f::Identity() + sinf(theta) * skew + (1 - cosf(theta)) * skew * skew;
}

Eigen::Matrix4f transformationMat(const Eigen::Vector3f& r, const Eigen::Vector3f& t) {
  Eigen::Matrix4f g = Eigen::Matrix4f::Identity();
  g.topLeftCorner(3, 3) = rotationMat(r);
  g(Eigen::seqN(0, 3), 3) = t;
  return g;
}

std::vector<CGL::Vector3D> convertVectors(const Eigen::Matrix<float, -1, 1>& pos) {
  int nPoints = pos.rows() / 3;
  std::vector<CGL::Vector3D> out;
  out.reserve(nPoints);
  for (int i = 0; i < nPoints; i++) {
    Eigen::Vector3f v = pos.middleRows<3>(3 * i);
    out.push_back(convertVector(v));
  }

  return out;
}

CGL::Vector3D convertVector(const Eigen::Vector3f& pos) {
  return CGL::Vector3D(pos(0, 0), pos(1, 0), pos(2, 0));
}

Eigen::Matrix<float, -1, 1> convertVectors(const std::vector<CGL::Vector3D>& pos) {
  Eigen::Matrix<float, -1, 1> out;
  out.resize(3 * pos.size(), 1);
  for (int i = 0; i < pos.size(); i++) {
    out.middleRows<3>(3 * i) = convertVector(pos[i]);
  }

  return out;
}

Eigen::Vector3f convertVector(const CGL::Vector3D& pos) {
  return Eigen::Vector3f(pos.x, pos.y, pos.z);
}
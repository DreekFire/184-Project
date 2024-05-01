#ifndef JANSEN_INTERNAL_H
#define JANSEN_INTERNAL_H

#include <vector>
#include <CGL/vector3D.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include "../collision/collisionObject.h"

typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixF;

class Jansen {
public:
  // only 1 coord because extrinsic translation and rotation are handled specially
  constexpr static int nLegs = 5;
  constexpr static int nCoords = 1;
  constexpr static int nPoints = nLegs * 14;

  Jansen();
  Jansen(Eigen::Matrix<float, nCoords, 1> q,
    Eigen::Matrix<float, nCoords, 1> qdot,
    Eigen::Vector3f translation,
    Eigen::Matrix3f rotation);

  void reset();
  // convert internal coords q to cartesian coords and calculate derivatives
  void resolve();
  // step the simulation using Lagrangian mechanics
  void simulate(double frames_per_sec, double simulation_steps,
    const std::vector<CGL::Vector3D>& external_accelerations,
    const std::vector<CollisionObject*>* const collision_objects,
    const std::vector<CGL::Vector3D>& external_forces,
    const std::vector<int>& force_idxs);
  void step(const std::vector<CGL::Vector3D>& forces, const std::vector<int>& idxs, double dt);
  void solveCollisions(const std::vector<CGL::Vector3D>& forces, const std::vector<float>& depths, const std::vector<int>& idxs, double dt);
  // convert the flat vector representation p to a vector of Vector3Ds
  CGL::Vector3D position(int i) const;
  CGL::Vector3D lastPosition(int i) const;

  std::vector<CGL::Vector3D> convertPositions() const;

  float getKE() {
    Eigen::Matrix<float, 3, nPoints> totalVel = v.reshaped(3, nPoints).colwise() + velocity;
    return pow(totalVel.sum(), 2);
  }

  const static Eigen::Matrix<float, 2, 3> fixedPos;
  const static std::vector<std::vector<int>> resolutionOrder;
  const static std::vector<std::vector<float>> linkLengths;

  // external mechanics
  // global translation and velocity (of CoM)
  Eigen::Vector3f translation;
  Eigen::Vector3f velocity;
  // angular momentum about CoM
  Eigen::Vector3f angularMomentum;
  // current rotation matrix
  Eigen::Matrix3f rotation;
  // inverse of inertia tensor
  Eigen::Matrix3f inverseMoment;

  // internal mechanics
  // generalized coordinates and their velocities
  Eigen::Matrix<float, nCoords, 1> q;
  Eigen::Matrix<float, nCoords, 1> qdot;
  // cartesian coordinates of each point mass (relative to CoM)
  Eigen::Matrix<float, 3 * nPoints, 1> p;
  Eigen::Matrix<float, 3 * nPoints, 1> v;
  Eigen::Matrix<float, 3 * nPoints, 1> vLast;
  std::vector<CGL::Vector3D> positions;
  std::vector<CGL::Vector3D> lastPositions;
  // derivatives
  Eigen::Matrix<float, 3 * nPoints, nCoords> dpdq;
  Eigen::Matrix<float, 3 * nPoints, nCoords> ddtdpdq;

  Eigen::Matrix<float, nCoords, 1> qInit;
  Eigen::Matrix<float, nCoords, 1> qdotInit;
  Eigen::Vector3f translationInit;
  Eigen::Matrix3f rotationInit;
};

#endif
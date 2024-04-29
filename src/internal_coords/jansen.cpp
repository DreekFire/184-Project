#include <iostream>

#include "jansen.h"

#include "CGL/vector2D.h"
#include "../linkageUtils.h"
#include "../pointMass.h"
#include "../collision/collisionObject.h"

const std::vector<std::vector<float>> Jansen::linkLengths = {
    {4.15f, 5.00f},
    {4.01f, 5.58f},
    {3.93f, 6.19f},
    {3.94f, 3.67f},
    {6.57f, 4.90f},

    {4.15f, 5.00f},
    {4.01f, 5.58f},
    {3.93f, 6.19f},
    {3.94f, 3.67f},
    {6.57f, 4.90f},
};

// 0 -> 2, 1 -> 0, 2 -> 1
const std::vector<std::vector<int>> Jansen::resolutionOrder = {
    {3, 2, 1, 1},
    {4, 2, 3, 1},
    {6, 2, 1, -1},
    {5, 4, 6, -1},
    {7, 5, 6, -1},

    {9, 8, 1, -1},
    {10, 8, 9, -1},
    {12, 8, 1, 1},
    {11, 10, 12, 1},
    {13, 11, 12, 1},
};

Jansen::Jansen() {
  reset();
}

void Jansen::reset() {
  q.fill(0);
  qdot.fill(1);
  translation = Eigen::Vector3f(0, 10, 0);
  velocity.setZero();
  rotation.setIdentity();

  pLast.setZero();
  vLast.setZero();
  resolve(0);
  angularMomentum.setZero();
}

void Jansen::resolve(double dt) {
  Eigen::Matrix<float, 3, nPoints> pos;
  pos.setZero();
  pos.col(1) = Eigen::Vector3f(1.5 * cosf(q(0)), 1.5 * sinf(q(0)), 0);
  pos.col(2) = Eigen::Vector3f(-3.8, -0.782, 0);
  pos.col(8) = Eigen::Vector3f(3.8, -0.782, 0);

  v.setZero();

  Eigen::Matrix<float, 3 * nPoints, nCoords> lastDpDq = dpdq;
  dpdq.setZero();
  dpdq.middleRows<3>(3) = Eigen::Vector3f(-1.5 * sinf(q(0)), 1.5 * cosf(q(0)), 0);

  ddtdpdq.setZero();
  ddtdpdq.middleRows<3>(3) = Eigen::Vector3f(-1.5 * cosf(q(0)), -1.5 * sinf(q(0)), 0) * qdot(0);

  // coordinates are [t, r, theta]: global translation, global rotation (axis-magnitude-angle), crankshaft angle
  for (int i = 0; i < resolutionOrder.size(); i++) {
    int idxR = resolutionOrder[i][0];
    int idx1 = resolutionOrder[i][1];
    int idx2 = resolutionOrder[i][2];
    Eigen::Vector3f s = intersectPointDistances(
      pos.col(idx2) - pos.col(idx1),
      linkLengths[i][0], linkLengths[i][1],
      Eigen::Vector3f(0, 0, resolutionOrder[i][3])
    );

    pos.col(idxR) = s + pos.col(idx1);

    Eigen::Matrix<float, 3, nCoords> vels = intersectPointVelocities(
      pos.col(idx2) - pos.col(idx1),
      dpdq.middleRows<3>(3 * idx2) - dpdq.middleRows<3>(3 * idx1),
      s
    );

    dpdq.middleRows<3>(3 * idxR) = dpdq.middleRows<3>(3 * idx1) + vels;

    v.middleRows<3>(3 * idxR) = dpdq.middleRows<3>(3 * idxR) * qdot;

    Eigen::Matrix<float, 3, nCoords> accels = intersectPointAccelerations(
      pos.col(idx2) - pos.col(idx1),
      v.middleRows<3>(3 * idxR) - v.middleRows<3>(3 * idx1),
      dpdq.middleRows<3>(3 * idxR) - dpdq.middleRows<3>(3 * idx1),
      ddtdpdq.middleRows<3>(3 * idx2) - ddtdpdq.middleRows<3>(3 * idx1),
      s
    );

    ddtdpdq.middleRows<3>(3 * idxR) = accels + ddtdpdq.middleRows<3>(3 * idx1);
  }
  if (pos.hasNaN() || dpdq.hasNaN() || ddtdpdq.hasNaN()) {
    std::cout << "NaNi!?" << std::endl;
  }
  pos = pos.colwise() - pos.rowwise().sum() / nPoints;
  p = pos.reshaped(3 * nPoints, 1);

  Eigen::Vector3f dL(0, 0, 0);
  for (int i = 0; i < nPoints; i++) {
    dL += pos.col(i).cross(v.middleRows<3>(3 * i) - vLast.middleRows<3>(3 * i));
  }
  angularMomentum -= dL;
  Eigen::Matrix3f moment = pos.squaredNorm() * Eigen::Matrix3f::Identity() - pos * pos.transpose();

  // fast inverse for PD matrices, doesn't really matter since it's just 3x3 but it feels cool
  inverseMoment = moment.llt().solve(Eigen::Matrix3f::Identity());
}

void Jansen::solveCollisions(const std::vector<CGL::Vector3D>& forces, const std::vector<float>& depths, const std::vector<int>& idxs, double dt) {
  /*// get force response
  // response is force projected onto columns of dpdq, but we handle extrinsic translation/rotation separately so we need to add that back.
  Eigen::Matrix<float, 3 * nPoints, 3> tResponse;
  Eigen::Matrix<float, 3 * nPoints, 3> rResponse;
  int nForces = forces.size();
  for (int i = 0; i < nPoints; i++) {
    tResponse.middleRows<3>(3 * i) = Eigen::Matrix3f::Identity();
    rResponse.middleRows<3>(3 * i) = -skewMat(p.middleRows<3>(3 * i));
  }
  Eigen::Matrix<float, 3 * nPoints, nCoords + 6> space;
  space << tResponse, rResponse, dpdq;
  Eigen::ColPivHouseholderQR<Eigen::Matrix<float, 3 * nPoints, nCoords + 6>> qr(space);
  Eigen::Matrix<float, 3 * nPoints, nCoords + 6> Q = qr.householderQ().setLength(nCoords + 6) * Eigen::Matrix<float, 3 * nPoints, nCoords + 6>::Identity();
  // nCoords+6 x nForces
  Eigen::Matrix<float, nCoords + 6, -1> qResponses;
  qResponses.setZero(nCoords + 6, nForces);
  // 3*nPoints x nForces
  Eigen::Matrix<float, 3 * nPoints, -1> responses;
  responses.setZero(3 * nPoints, nForces);
  int rank = qr.rank();
  for (int i = 0; i < nForces; i++) {
    qResponses.col(i) = Q.middleRows<3>(3 * idxs[i]).transpose() * convertVector(forces[i]); // currently actually R * qResponses
    responses.col(i) = Q * qResponses.col(i);
    // calculate R^{-1} * qResponses, still faster than fully multiplying out dpdqs
    qResponses.col(i) = qr.matrixR().topLeftCorner(nCoords + 6, nCoords + 6).triangularView<Eigen::Upper>().solve(qResponses.col(i));
  }*/

  // get force response
  // response is force projected onto columns of dpdq, but we handle extrinsic translation/rotation separately so we need to add that back.
  int nForces = forces.size();
  Eigen::Matrix<float, 3, -1> f = convertVectors(forces).reshaped(3, nForces);
  Eigen::Matrix<float, 3, -1> rResponse;
  rResponse.resize(3, nForces);

  for (int i = 0; i < nForces; i++) {
    rResponse.col(i) = p.middleRows<3>(3 * idxs[i]).cross(f.col(i));
  }

  Eigen::Matrix<float, 3, -1> wResponse = inverseMoment * rResponse;
  
  // q response is pseudoinverse(dpdq) * forces, p response is dpdq * pseudoinverse(dpdq) * forces, which is the projection onto the columns of dpdq
  // nCoords x 1 = (nCoords x nCoords)^-1 * nCoords x 3 * 3 x 1
  Eigen::Matrix<float, nCoords, -1> qResponses;
  qResponses.resize(nCoords, nForces);
  for (int i = 0; i < nForces; i++) {
    qResponses.col(i) = (dpdq.transpose() * dpdq).llt().solve(
      dpdq.middleRows<3>(3 * idxs[i]).transpose()
      * f.col(i));
  }
  // we want response[i].dot(forces[i])
  // (i, j) is the contribution to constrainti from force j
  Eigen::MatrixXf jacobian = Eigen::MatrixXf::Identity(nForces, nForces);
  for (int i = 0; i < nForces; i++) {
    jacobian.row(i) += f.col(i).transpose() * dpdq.middleRows<3>(3 * idxs[i]) * qResponses; // 1 x 3 * 3 x nCoords * nCoords x nForces
  }

  jacobian += f.transpose() * f / (nPoints * nPoints);
  for (int i = 0; i < nForces; i++) {
    for (int j = 0; j < nForces; j++) {
      jacobian(i, j) += wResponse.col(j).cross(p.middleRows<3>(3 * idxs[i])).dot(f.col(i));
    }
  }

  jacobian *= dt * dt;

  // return responses;

  /*Eigen::MatrixXf jacobian;
  jacobian.setZero(nForces, nForces);
  // dsigma/dlambda = response * grad * dt^2

  for (int i = 0; i < nForces; i++) {
      for (int j = 0; j < nForces; j++) {
          // how much force i contributes to constraint j
          int cIdx = idxs[j];
          jacobian(i, j) = responses.block<3, 1>(3 * cIdx, i).dot(convertVector(forces[j])) * dt * dt;
      }
  }*/

  Eigen::VectorXf sigma;
  sigma.resize(nForces);
  for (int i = 0; i < nForces; i++) {
    sigma(i) = depths[i];
  }
  Eigen::VectorXf lambda = jacobian.colPivHouseholderQr().solve(sigma);

  Eigen::Matrix<float, nCoords, 1> qAccel = qResponses * lambda;
  qdot += qAccel * dt;
  q += qAccel * dt * dt;

  for (int i = 0; i < nForces; i++) {
    f.col(i) *= lambda(i);
  }
  Eigen::Vector3f globalForce = f.rowwise().sum();

  Eigen::Vector3f torque = Eigen::Vector3f::Zero();
  for (int i = 0; i < nForces; i++) {
    torque += rResponse.col(i) * lambda(i);
  }

  Eigen::Vector3f dv = globalForce * dt / nPoints;
  velocity += dv;
  angularMomentum += torque * dt;

  resolve(dt);

  rotation = rotationMat(inverseMoment * torque * dt * dt) * rotation;
  translation += dv * dt;
}

void Jansen::step(const std::vector<CGL::Vector3D>& forces, const std::vector<int>& idxs, double dt) {
    // just using unit masses for now

    // using numerator convention
    // dKE/dv = mv^T
    // dKE/dqdot = mv^T dv/dqdot = mv^T dp/dq         1xM * MxN
    // dKE/dqdot = mv^T dp/dq = m (dp/dq qdot)^T dp/dq = m qdot^T dp/dq^T dp/dq    1xN * NxM * MxN
    // d/dt dKE/dqdot = m qddot^T dp/dq^T dp/dq + m qdot^T d/dt (dp/dq^T dp/dq)
    // = m qddot^T dp/dq^T dp/dq + m qdot^T (d/dt dpdq^T * dpdq + (d/dt dpdq^T * dpdq)^T)
    // 
    // d/dt dKE/dqdot_i = m dp/dq_i^T dp/dq_i qddot_i + m d/dt (dp/dq_i^T dp/dq_i) qdot_i = dP/dq_i
    // = m dp/dq_i^T dp/dq_i qddot_i + m (ddtdpdq_i^T * dpdq_i + (...)^T) qdot_i = dP/dq_i
    // N = scalar * M * M * N + scalar * (M * M + ...) * scalar = dP/dq_i 
    // dP/dq = f^T * dpdq
    // 

    // R = qr.matrixR() upper triangular
    // mass matrix = R^T Q^T Q R = R^T R already Cholesky decomposed
    Eigen::LLT<Eigen::Matrix<float, nCoords, nCoords>> mass(dpdq.transpose() * dpdq);
    
    Eigen::Matrix<float, nCoords, nCoords> coriolis = ddtdpdq.transpose() * dpdq;
    coriolis += coriolis.transpose();

    // centrifugal force = -m w x (w x r)
    // coriolis force = -2m w x v
    Eigen::Vector3f w = inverseMoment * angularMomentum;
    Eigen::Matrix3f wHat = skewMat(w);
    Eigen::Matrix<float, 3, nPoints> f = -2 * wHat * v.reshaped(3, nPoints) - wHat * wHat * p.reshaped(3, nPoints);

    Eigen::Matrix<float, 3, -1> forcesMat = convertVectors(forces).reshaped(3, nPoints);
    Eigen::Vector3f globalForce = forcesMat.rowwise().sum();

    Eigen::Vector3f torque = Eigen::Vector3f::Zero();
    // flatten forces into f
    for (int i = 0; i < forces.size(); i++) {
      f.col(idxs[i]) += forcesMat.col(i);
      torque += p.middleRows<3>(3 * idxs[i]).cross(forcesMat.col(i));
    }

    Eigen::Matrix<float, 1, nCoords> gravity = f.reshaped(1, 3 * nPoints) * dpdq; // 1xM * MxN
    // solve using TriangularView uses forward/backward substitution - faster than computing full R^T R and Cholesky decomposition
    Eigen::Matrix<float, nCoords, 1> qAccel = mass.solve((gravity - qdot.transpose() * coriolis).transpose());

    qdot += qAccel * dt;
    q += qdot * dt;

    velocity += globalForce * dt / nPoints;

    angularMomentum += torque * dt;

    resolve(dt);

    translation += velocity * dt;
    w = inverseMoment * angularMomentum;
    rotation = rotationMat(w * dt) * rotation;
    if (rotation.hasNaN()) {
      std::cout << "NaNi!?" << std::endl;
    }
}

void Jansen::simulate(double frames_per_sec, double simulation_steps,
  const std::vector<CGL::Vector3D>& external_accelerations,
  const std::vector<CollisionObject*>* const collision_objects) {

  CGL::Vector3D total_external_acceleration(0);
  std::vector<CGL::Vector3D> total_forces(nPoints, CGL::Vector3D(0));
  for (const CGL::Vector3D& a : external_accelerations) {
    total_external_acceleration += a;
  }
  for (CGL::Vector3D& f : total_forces) {
    f += total_external_acceleration;
  }
  std::vector<int> idxs;
  idxs.reserve(nPoints);
  for (int i = 0; i < nPoints; i++) {
    idxs.push_back(i);
  }

  double dt = 1 / (frames_per_sec * simulation_steps);

  pLast = p;
  vLast = v;

  this->step(total_forces, idxs, dt);

  // check for collisions

  std::vector<CGL::Vector3D> collision_offsets;
  std::vector<float> collision_depths;
  std::vector<int> collision_idxs;
  for (int i = 0; i < nPoints; i++) {
    CGL::Vector3D pos = this->position(i);
    // simple y-check just for testing
    if (pos.y < 0) {
      CGL::Vector3D diff = CGL::Vector3D(0, -pos.y, 0);
      float d = diff.norm();
      collision_depths.push_back(d);
      collision_offsets.push_back(diff / d);
      collision_idxs.push_back(i);
    }

    /*PointMass pm(pos, false);
    for (CollisionObject* co : *collision_objects) {
      pm.last_position = this->lastPosition(i);
      co->collide(pm);
    }
    CGL::Vector3D diff = pm.position - pos;
    if (diff.norm2() > 0.001) {
      float d = diff.norm();
      collision_depths.push_back(d);
      collision_offsets.push_back(diff / d);
      collision_idxs.push_back(i);
    }*/
  }

  if (collision_depths.size() > 0) {
    this->solveCollisions(collision_offsets, collision_depths, collision_idxs, dt);
  }
}

std::vector<CGL::Vector3D> Jansen::positions() const {
  return convertVectors(((rotation * p.reshaped(3, nPoints)).colwise() + translation).reshaped());
}

std::vector<CGL::Vector3D> Jansen::lastPositions() const {
  return convertVectors(((rotation * pLast.reshaped(3, nPoints)).colwise() + translation).reshaped());
}

CGL::Vector3D Jansen::position(int i) const {
  return convertVector(rotation * p.middleRows<3>(3 * i) + translation);
}

CGL::Vector3D Jansen::lastPosition(int i) const {
  return convertVector(rotation * pLast.middleRows<3>(3 * i) + translation);
}

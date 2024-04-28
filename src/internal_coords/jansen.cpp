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
};

const std::vector<std::vector<int>> Jansen::resolutionOrder = {
    {3, 0, 2, 1},
    {4, 0, 3, 1},
    {6, 0, 2, -1},
    {5, 4, 6, -1},
    {7, 5, 6, -1},
};

Jansen::Jansen() {
  q.fill(0);
  qdot.fill(1);
  translation.setZero();
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
  pos.col(1) = Eigen::Vector3f(3.8, 0.782, 0);
  pos.col(2) = pos.col(1) + Eigen::Vector3f(1.5 * cosf(q(0)), 1.5 * sinf(q(0)), 0);

  v.setZero();

  Eigen::Matrix<float, 3 * nPoints, nCoords> lastDpDq = dpdq;
  dpdq.setZero();
  dpdq.middleRows<3>(6) = Eigen::Vector3f(-1.5 * sinf(q(0)), 1.5 * cosf(q(0)), 0);

  ddtdpdq.setZero();
  ddtdpdq.middleRows<3>(6) = Eigen::Vector3f(-1.5 * cosf(q(0)), -1.5 * sinf(q(0)), 0) * qdot(0);

  // coordinates are [t, r, theta]: global translation, global rotation (axis-magnitude-angle), crankshaft angle
  for (int i = 0; i < resolutionOrder.size(); i++) {
    int idxR = resolutionOrder[i][0];
    int idx1 = resolutionOrder[i][1];
    int idx2 = resolutionOrder[i][2];
    // printf("intersecting %d, %d to produce point %d\n", idx1, idx2, idxR);
    // std::cout << "p1: " << position[idx1] << " p2: " << position[idx2] << " r1: " << linkLengths[i][0] << " r2: " << linkLengths[i][1] << std::endl;
    Eigen::Vector3f s = intersectPointDistances(
      pos.col(idx2) - pos.col(idx1),
      linkLengths[i][0], linkLengths[i][1],
      Eigen::Vector3f(0, 0, resolutionOrder[i][3])
    );

    pos.col(idxR) = s + pos.col(idx1);

    // printf("intersecting %d, %d to produce point %d\n", idx1, idx2, idxR);
    // std::cout << "p1: " << position[idx1] << " p2: " << position[idx2] << " r1: " << linkLengths[i][0] << " r2: " << linkLengths[i][1] << std::endl;
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
    
    /*if (pos.maxCoeff() > 100 || dpdq.maxCoeff() > 100 || ddtdpdq.maxCoeff() > 200) {
      std::cout << "uh oh" << std::endl;
    }*/
  }
  Eigen::Matrix<float, 3 * nPoints, nCoords> dpdqDiff = dpdq - lastDpDq;
  // std::cout << "accel diff: " << dpdqDiff - ddtdpdq / (90 * 30) << std::endl;
  Eigen::Matrix<float, 3 * nPoints, 1> pDiff = p - pLast;
  // std::cout << "vel diff: " << pDiff - (dpdq * qdot / (90 * 30)) << std::endl;
  if (pos.hasNaN() || dpdq.hasNaN() || ddtdpdq.hasNaN()) {
    std::cout << "NaNi!?" << std::endl;
  }
  pos = pos.colwise() - pos.rowwise().sum() / nPoints;
  p = pos.reshaped(3 * nPoints, 1);

  Eigen::Vector3f torque(0, 0, 0);
  for (int i = 0; i < nPoints; i++) {
    torque += pos.col(i).cross(v.middleRows<3>(3 * i) - vLast.middleRows<3>(3 * i));
  }
  angularMomentum -= torque * dt;
  Eigen::Matrix3f moment = pos.squaredNorm() * Eigen::Matrix3f::Identity() - pos * pos.transpose();

  // fast inverse for PD matrices, doesn't really matter since it's just 3x3 but it feels cool
  inverseMoment = moment.llt().solve(Eigen::Matrix3f::Identity());

        /*dpdq[idxR] = intersectPointVelocities(
            pos[idx2] - pos[idx1],
            { dpdq[idx2][0] - dpdq[idx1][0] },
            p
        );
        CGL::Vector2D vel(0, 0);
        for (int i = 0; i < dpdq[idxR].size(); i++) {
            vel += (dpdq[idx2][i] - dpdq[idx1][i]) * qdot[i];
        }
        ddtdpdq[idxR] = intersectPointAccelerations(
            pos[idx2] - pos[idx1],
            vel,
            { ddtdpdq[idx2][0] - ddtdpdq[idx1][0] },
            p
        );*/
        // { velocity[idx2][0] - velocity[idx1][0] },
        // pos.col(idxR) = s + pos.col(idx1);
        /*for (int i = 0; i < qdot.size(); i++) {
            dpdq[idxR][i] += dpdq[idx1][i];
            ddtdpdq[idxR][i] += ddtdpdq[idx1][i];
        }*/
        // std::cout << position[idxR] << std::endl;
    // }
    /*CGL::Vector2D com = CGL::Vector2D(0, 0);
    for (CGL::Vector2D& p : pos) {
        com += p;
    }
    com /= pos.size();
    for (CGL::Vector2D& p : pos) {
        p -= com;
    }*/
}

void Jansen::solveCollisions(const std::vector<CGL::Vector3D>& forces, const std::vector<float>& depths, const std::vector<int>& idxs, double dt) {
    // get force response
    Eigen::ColPivHouseholderQR<Eigen::Matrix<float, 3 * nPoints, nCoords>> qr(dpdq);
    Eigen::Matrix<float, 3 * nPoints, nCoords> Q = qr.householderQ().setLength(nCoords) * Eigen::Matrix<float, 3 * nPoints, nCoords>::Identity();
    // nForces x nCoords
    Eigen::Matrix<float, Eigen::Dynamic, nCoords> qResponses = Eigen::Matrix<float, Eigen::Dynamic, nCoords>::Zero(forces.size(), nPoints);
    // nForces x 3*nPoints
    Eigen::Matrix<float, Eigen::Dynamic, 3 * nPoints> responses = Eigen::Matrix<float, Eigen::Dynamic, 3 * nPoints>::Zero(forces.size(), nPoints);
    int rank = qr.rank();
    for (int i = 0; i < forces.size(); i++) {
        qResponses.col(i) = Q.middleRows<3>(3 * idxs[i]).transpose() * convertVector(forces[i]); // currently actually R * qResponses
        responses.col(i) = Q * qResponses.col(i);
        qResponses.col(i) = qr.matrixR().triangularView<Eigen::Upper>().solve(qResponses.col(i)); // calculate R^{-1} * qResponses, still faster than fully multiplying out dpdqs
    }

    // return responses;
    // offsets are 3 x nForces
    // multiply offs

    int nForces = forces.size();
    Eigen::MatrixXf jacobian = Eigen::MatrixXf::Zero(nForces, nForces);

    // dsigma/dlambda = response * grad * dt^2

    for (int i = 0; i < nForces; i++) {
        for (int j = 0; j < nForces; j++) {
            // how much force i contributes to constraint j
            int cIdx = idxs[j];
            jacobian(i, j) = (dot(convertVector(responses(i, Eigen::seqN(3 * cIdx, 3))), forces[j])) * dt * dt;
        }
    }

    Eigen::VectorXf sigma;
    sigma.resize(depths.size());
    for (int i = 0; i < depths.size(); i++) {
      sigma(i) = depths[i];
    }
    Eigen::VectorXf lambda = -jacobian.colPivHouseholderQr().solve(sigma);

    Eigen::Matrix<float, nCoords, 1> qAccel = qResponses.transpose() * lambda;
    qdot += qAccel * dt;
    q += qAccel * dt * dt;

    Eigen::Matrix<float, 3, -1> f = convertVectors(forces).reshaped(3, nPoints) * lambda;
    Eigen::Vector3f globalForce = f.rowwise().sum();

    Eigen::Vector3f torque = Eigen::Vector3f::Zero();
    for (int i = 0; i < forces.size(); i++) {
      torque += p.middleRows<3>(3 * i).cross(f.col(i));
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
    // coriolis.setZero();
    // std::cout << "cor: " << coriolis << std::endl;

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
  /*for (const CGL::Vector3D& a : external_accelerations) {
    total_external_acceleration += a;
  }
  for (CGL::Vector3D& f : total_forces) {
    f += total_external_acceleration;
  }*/
  // total_forces[1] += Vector3D(0, 1, 0);
  // total_forces[2] += Vector3D(0, -1, 0);
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

  /*std::vector<CGL::Vector3D> collision_offsets;
  std::vector<float> collision_depths;
  std::vector<int> collision_idxs;
  for (int i = 0; i < nPoints; i++) {
    CGL::Vector3D pos = this->position(i);
    PointMass pm(pos, false);
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
    }
  }

  this->solveCollisions(collision_offsets, collision_depths, collision_idxs, dt);*/
}

std::vector<CGL::Vector3D> Jansen::positions() const {
  return convertVectors(((rotation * p.reshaped(3, nPoints)).colwise() + translation).reshaped());
}

std::vector<CGL::Vector3D> Jansen::lastPositions() const {
  return convertVectors(((rotation * pLast.reshaped(3, nPoints)).colwise() + translation).reshaped());
}

CGL::Vector3D Jansen::position(int i) const {
  return convertVector(p.middleRows<3>(3 * i) + translation);
}

CGL::Vector3D Jansen::lastPosition(int i) const {
  return convertVector(pLast.middleRows<3>(3 * i) + translation);
}

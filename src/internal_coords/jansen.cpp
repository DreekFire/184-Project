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

// 0 -> 2, 1 -> 0, 2 -> 1
const std::vector<std::vector<int>> Jansen::resolutionOrder = {
    {3, 2, 1, 1},
    {4, 2, 3, 1},
    {6, 2, 1, -1},
    {5, 4, 6, -1},
    {7, 5, 6, -1},
};

Jansen::Jansen() : qInit(0), qdotInit(0), translationInit(Eigen::Vector3f(0, 0, 0)), rotationInit(Eigen::Matrix3f::Identity()) {
  positions = std::vector<CGL::Vector3D>(nPoints);
  lastPositions = std::vector<CGL::Vector3D>(nPoints);
  reset();
}

Jansen::Jansen(Eigen::Matrix<float, nCoords, 1> q,
  Eigen::Matrix<float, nCoords, 1> qdot,
  Eigen::Vector3f translation,
  Eigen::Matrix3f rotation) : qInit(q), qdotInit(qdot), translationInit(translation), rotationInit(rotation) {
  velocity = Eigen::Vector3f(0, 0, 0);

  positions = std::vector<CGL::Vector3D>(nPoints);
  lastPositions = std::vector<CGL::Vector3D>(nPoints);
  reset();
}

void Jansen::reset() {
  q = qInit;
  qdot = qdotInit;
  translation = translationInit;
  velocity.setZero();
  rotation = rotationInit;

  vLast.setZero();
  resolve();
  positions = convertPositions();
  std::copy(positions.begin(), positions.end(), lastPositions.begin());
  angularMomentum.setZero();
}

void Jansen::resolve() {
  Eigen::Matrix<float, 3, nPoints> pos;
  pos.setZero();
  v.setZero();

  Eigen::Matrix<float, 3 * nPoints, nCoords> lastDpDq = dpdq;
  dpdq.setZero();
  ddtdpdq.setZero();

  int pPerSet = 14;
  float spacing = 6.0f;
  float scaling = 0.25f;
  for (int l = 0; l < nLegs; l++) {
    float qEff = q(0) + 2 * PI / nLegs * l;
    pos.col(pPerSet * l) = Eigen::Vector3f(0, 0, spacing * l);
    pos.col(pPerSet * l + 1) = Eigen::Vector3f(1.5 * cosf(qEff), 1.5 * sinf(qEff), spacing * l);
    dpdq.block<3, 1>(3 * pPerSet * l + 3, 0) = Eigen::Vector3f(-1.5 * sinf(qEff), 1.5 * cosf(qEff), 0);
    ddtdpdq.block<3, 1>(3 * pPerSet * l + 3, 0) = Eigen::Vector3f(-1.5 * cosf(qEff), -1.5 * sinf(qEff), 0) * qdot(0);
    for (int p = 0; p < 2; p++) {
      int side = p == 0 ? 1 : -1;
      pos.col(pPerSet * l + 6 * p + 2) = Eigen::Vector3f(-side * 3.8, -0.782, spacing * l);

      for (int i = 0; i < resolutionOrder.size(); i++) {
        int idx1 = resolutionOrder[i][1] + 6 * p + pPerSet * l;
        int idx2 = resolutionOrder[i][2];
        if (idx2 != 1) {
          idx2 += 6 * p;
        }
        idx2 += pPerSet * l;
        int idxR = resolutionOrder[i][0] + 6 * p + pPerSet * l;
        Eigen::Vector3f s = intersectPointDistances(
          pos.col(idx2) - pos.col(idx1),
          linkLengths[i][0], linkLengths[i][1],
          Eigen::Vector3f(0, 0, side * resolutionOrder[i][3])
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
    }
  }

  // coordinates are [t, r, theta]: global translation, global rotation (axis-magnitude-angle), crankshaft angle
  
  if (pos.hasNaN() || dpdq.hasNaN() || ddtdpdq.hasNaN()) {
    std::cout << "NaNi!?" << std::endl;
  }
  pos = scaling * (pos.colwise() - pos.rowwise().mean());
  p = pos.reshaped(3 * nPoints, 1);
  v = scaling * (v.reshaped(3, nPoints).colwise() - v.reshaped(3, nPoints).rowwise().mean()).reshaped();
  dpdq *= scaling;
  ddtdpdq *= scaling;

  for (int i = 0; i < nCoords; i++) {
    Eigen::Vector3f meanDp = dpdq.col(i).reshaped(3, nPoints).rowwise().mean();
    dpdq.col(i) = (dpdq.col(i).reshaped(3, nPoints).colwise() - meanDp).reshaped();

    Eigen::Vector3f meanDdp = ddtdpdq.col(i).reshaped(3, nPoints).rowwise().mean();
    ddtdpdq.col(i) = (ddtdpdq.col(i).reshaped(3, nPoints).colwise() - meanDdp).reshaped();
  }

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
    rResponse.col(i) = (rotation * p.middleRows<3>(3 * idxs[i])).cross(f.col(i));
  }

  Eigen::Matrix<float, 3, -1> wResponse = inverseMoment * rResponse;

  // q response is pseudoinverse(dpdq) * forces, p response is dpdq * pseudoinverse(dpdq) * forces, which is the projection onto the columns of dpdq
  // nCoords x 1 = (nCoords x nCoords)^-1 * nCoords x 3 * 3 x 1
  Eigen::Matrix<float, nCoords, -1> qResponses;
  qResponses.resize(nCoords, nForces);
  Eigen::LLT<Eigen::Matrix<float, nCoords, nCoords>> mass(dpdq.transpose() * dpdq);
  for (int i = 0; i < nForces; i++) {
    qResponses.col(i) = mass.solve(
      (rotation * dpdq.middleRows<3>(3 * idxs[i])).transpose()
      * f.col(i));
  }

  Eigen::Matrix<float, 3 * nPoints, -1> pResponses = dpdq * qResponses;;
  // we want response[i].dot(forces[i])
  // (i, j) is the contribution to constraint i from force j
  Eigen::MatrixXf jacobian;
  jacobian.resize(nForces, nForces);
  for (int i = 0; i < nForces; i++) {
    jacobian.row(i) = f.col(i).transpose() * rotation * dpdq.middleRows<3>(3 * idxs[i]) * qResponses; // 1 x 3 * 3 x nCoords * nCoords x nForces
  }

  jacobian += f.transpose() * f / nPoints;
  for (int i = 0; i < nForces; i++) {
    for (int j = 0; j < nForces; j++) {
      jacobian(i, j) += wResponse.col(j).cross(rotation * p.middleRows<3>(3 * idxs[i])).dot(f.col(i));
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
  std::vector<int> posIdxs;
  posIdxs.reserve(nForces);
  for (int i = 0; i < nForces; i++) {
    posIdxs.push_back(i);
  }
  Eigen::VectorXf lambda = jacobian.colPivHouseholderQr().solve(sigma);
  while (true) {
    std::vector<int> newIdxs;
    std::vector<int> keepIdxs;
    bool hasNegative = false;
    for (int i = 0; i < posIdxs.size(); i++) {
      if (lambda(posIdxs[i]) > 0) {
        newIdxs.push_back(posIdxs[i]);
        keepIdxs.push_back(i);
      }
      else {
        hasNegative = true;
      }
    }
    posIdxs = newIdxs;

    if (!hasNegative || posIdxs.empty()) {
      break;
    }
    jacobian = jacobian(keepIdxs, keepIdxs).eval();
    lambda.setZero();
    lambda(posIdxs) = jacobian.colPivHouseholderQr().solve(sigma(posIdxs));
  }

  if (posIdxs.empty()) {
    return;
  }

  /*for (int i = 0; i < nForces; i += 3) {
    float cone = abs(0.1f * lambda(i + 1));
    lambda(i) = min(cone, lambda(i));
    lambda(i + 2) = min(cone, lambda(i + 2));
  }*/

  /*for (int i = 0; i < nForces; i++) {
    float cone = abs(0.2f * lambda(i));
    CGL::Vector3D vParallel = positions[idxs[i]] - lastPositions[idxs[i]];
    vParallel -= dot(vParallel, forces[i]) * forces[i];
    // todo: simulate friction, but don't overshoot the amount of force needed for a complete stop
    // get qResponse and rResponse of frictional force, calculate amount needed to come to a stop, clamp by mu
    // CGL::Vector3D fParallel = 
    // OR: restrict depths to friction cone
  }*/

  float mu = 0.5;

  std::vector<int> trueIdxs;
  for (int i : posIdxs) {
    trueIdxs.push_back(idxs[i]);
  }
  Eigen::Matrix<float, 3, -1> globalVels = rotation * v.reshaped(3, nPoints)(Eigen::all, trueIdxs);
  for (int i = 0; i < posIdxs.size(); i++) {
    globalVels.col(i) =
      ((inverseMoment * angularMomentum)
        .cross(rotation * p.reshaped(3, nPoints).col(trueIdxs[i]))
      + (globalVels.col(i) + velocity)).normalized();
  }
  globalVels.row(1).setZero();
  // todo: make perpendicular to force
  
  Eigen::Matrix<float, nCoords, -1> qFrictionResponses;
  qFrictionResponses.resize(nCoords, posIdxs.size());
  for (int i = 0; i < posIdxs.size(); i++) {
    qFrictionResponses.col(i) = mass.solve(
      (rotation * dpdq.middleRows<3>(3 * trueIdxs[i])).transpose()
      * -globalVels.col(i));
  }

  Eigen::Matrix<float, 3, -1> rFrictionResponse;
  rFrictionResponse.resize(3, posIdxs.size());

  for (int i = 0; i < posIdxs.size(); i++) {
    rFrictionResponse.col(i) = (rotation * p.middleRows<3>(3 * trueIdxs[i]))
      .cross(-globalVels.col(i));
  }

  Eigen::Matrix<float, nCoords, 1> qAccel =
    qResponses(Eigen::all, posIdxs) * lambda(posIdxs)
    + mu * qFrictionResponses * lambda(posIdxs);
  qdot += qAccel * dt;
  q += qAccel * dt * dt;

  Eigen::Vector3f globalForce =
    f(Eigen::all, posIdxs) * lambda(posIdxs)
    + mu * -globalVels * lambda(posIdxs);

  Eigen::Vector3f torque =
    rResponse(Eigen::all, posIdxs) * lambda(posIdxs)
    + mu * rFrictionResponse * lambda(posIdxs);

  Eigen::Vector3f dv = globalForce * dt / nPoints;

  if (lambda.hasNaN() || !lambda.allFinite() || dv.maxCoeff() > 10 || dv.minCoeff() < -10
     || lambda.maxCoeff() > 1e6 || lambda.minCoeff() < -1e6) {
    std::cout << "NaNi!?" << std::endl;
  }

  /*double yBefore = 0;
  std::vector<CGL::Vector3D> posBefore = positions;
  for (CGL::Vector3D& vp : posBefore) {
    yBefore = min(yBefore, vp.y);
  }*/
  velocity += dv;
  angularMomentum += torque * dt;

  resolve();

  rotation = rotationMat(inverseMoment * torque * dt * dt) * rotation;
  translation += dv * dt;
  /*double yAfter = 0;
  std::vector<CGL::Vector3D> posAfter = convertPositions();
  for (CGL::Vector3D& vp : posAfter) {
    yAfter = min(yAfter, vp.y);
  }
  if (yAfter < yBefore) {
    std::cout << "ruh roh" << std::endl;
  }*/
  if (velocity.hasNaN() || angularMomentum.hasNaN()) {
    std::cout << "NaNi!?" << std::endl;
  }
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
  if (velocity.hasNaN() || velocity.maxCoeff() > 50 || angularMomentum.maxCoeff() > 10000) {
    std::cout << "NaNi!?" << std::endl;
  }
  Eigen::LLT<Eigen::Matrix<float, nCoords, nCoords>> mass(dpdq.transpose() * dpdq);
    
  Eigen::Matrix<float, nCoords, nCoords> coriolis = ddtdpdq.transpose() * dpdq;
  coriolis += coriolis.transpose();
  // coriolis.setZero();

  // centrifugal force = -m w x (w x r)
  // coriolis force = -2m w x v
  Eigen::Vector3f w = inverseMoment * angularMomentum;
  Eigen::Matrix3f wHat = skewMat(w);
  Eigen::Matrix<float, 3, nPoints> f = -2 * wHat * rotation * v.reshaped(3, nPoints) - wHat * wHat * rotation * p.reshaped(3, nPoints);

  Eigen::Matrix<float, 3, -1> forcesMat = convertVectors(forces).reshaped(3, nPoints);
  Eigen::Vector3f globalForce = forcesMat.rowwise().sum();

  Eigen::Vector3f torque = Eigen::Vector3f::Zero();
  // flatten forces into f
  for (int i = 0; i < forces.size(); i++) {
    f.col(idxs[i]) += forcesMat.col(i);
    torque += rotation * p.middleRows<3>(3 * idxs[i]).cross(forcesMat.col(i));
  }

  Eigen::Matrix<float, 3, nPoints> fCentered = f.colwise() - f.rowwise().mean();

  Eigen::Matrix<float, 3 * nPoints, nCoords> dpdqGlobal;
  for (int i = 0; i < nPoints; i++) {
    dpdqGlobal.middleRows<3>(3 * i) = rotation * dpdq.middleRows<3>(3 * i);
  }

  Eigen::Matrix<float, 1, nCoords> gravity = fCentered.reshaped().transpose() * dpdqGlobal; // 1xM * MxN
  Eigen::Matrix<float, nCoords, 1> qAccel = mass.solve((gravity - qdot.transpose() * coriolis).transpose());

  if (qAccel.hasNaN() || qAccel.maxCoeff() > 1e5 || qAccel.minCoeff() < -1e5) {
    std::cout << "NaNi!?" << std::endl;
  }

  qdot += qAccel * dt;
  q += qdot * dt;

  velocity += globalForce * dt / nPoints;

  angularMomentum += torque * dt;

  resolve();

  translation += velocity * dt;
  w = inverseMoment * angularMomentum;
  rotation = rotationMat(w * dt) * rotation;
  if (rotation.hasNaN()) {
    std::cout << "NaNi!?" << std::endl;
  }
}

void Jansen::simulate(double frames_per_sec, double simulation_steps,
  const std::vector<CGL::Vector3D>& external_accelerations,
  const std::vector<CollisionObject*>* const collision_objects,
  const std::vector<CGL::Vector3D>& external_forces,
  const std::vector<int>& force_idxs) {

  CGL::Vector3D total_external_acceleration(0);
  std::vector<CGL::Vector3D> total_forces(nPoints, CGL::Vector3D(0));
  for (const CGL::Vector3D& a : external_accelerations) {
    total_external_acceleration += a;
  }
  for (int i = 0; i < external_forces.size(); i++) {
    total_forces[force_idxs[i]] += external_forces[i];
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

  vLast = v;

  this->step(total_forces, idxs, dt);
  // check for collisions

  lastPositions = positions;
  positions = convertPositions();

  std::vector<CGL::Vector3D> collision_forces;
  std::vector<float> collision_depths;
  std::vector<int> collision_idxs;
  PointMass pm(CGL::Vector3D(), false);
  for (int i = 0; i < nPoints; i++) {
    CGL::Vector3D pos = positions[i];
    // simple y-check just for testing
    pm.position = pos;
    pm.last_position = lastPositions[i];
    //if (pos.y < 0) {
      // p + (lp - p) / (lp.y - p.y) * (- p.y)
      //pm.position += (pm.last_position - pm.position) * min(max(float(- pm.position.y / (pm.last_position.y - pm.position.y)), 0.0f), 1.0f);
      // pm.position.y = 0;
    //}

    for (CollisionObject* co : *collision_objects) {
      co->collide(pm);
    }
    CGL::Vector3D diff = pm.position - pos;
    float dNorm = diff.norm();
    if (dNorm > 0) {
      // float clampFactor = min((0.1 * diff.y) / sqrt(diff.x * diff.x + diff.z * diff.z), 1.0);
      // diff.x *= clampFactor;
      // diff.z *= clampFactor;
      //for (int c = 0; c < 3; c++) { // haha c++
      // todo: constrain pv to be within friction cone, then pass entire pv
      collision_depths.push_back(dNorm);
      // collision_depths.push_back(abs(diff[c]));
      // CGL::Vector3D f = CGL::Vector3D();
      // f[c] = diff[c] > 0 ? 1 : -1;
      collision_forces.push_back(diff / dNorm);
      collision_idxs.push_back(i);
      //}
    }
    // maybe handle friction separately?
  }

  if (collision_depths.size() > 0) {
    this->solveCollisions(collision_forces, collision_depths, collision_idxs, dt);
  }

  velocity *= 1 - 5 * dt;
  qdot *= 1 - 5 * dt;
  angularMomentum *= 1 - 5 * dt;
  positions = convertPositions();
}

CGL::Vector3D Jansen::position(int i) const {
  return convertVector(rotation * convertVector(positions[i])) + convertVector(translation);
}

CGL::Vector3D Jansen::lastPosition(int i) const {
  return convertVector(rotation * convertVector(lastPositions[i])) + convertVector(translation);
}

std::vector<CGL::Vector3D> Jansen::convertPositions() const {
  return convertVectors(((rotation * p.reshaped(3, nPoints)).colwise() + translation).reshaped());
}

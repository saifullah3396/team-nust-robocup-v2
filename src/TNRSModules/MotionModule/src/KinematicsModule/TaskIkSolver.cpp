/**
 * @file MotionModule/include/KinematicsModule/TaskIkSolver.cpp
 *
 * This file implements the class TaskIkSolver
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 Jul 2018
 */

#include "MotionModule/include/KinematicsModule/Joint.h"
#include "MotionModule/include/KinematicsModule/KinematicsModule.h"
#include "MotionModule/include/KinematicsModule/TaskIkSolver.h"
#include "Utils/include/DebugUtils.h"
#include "Utils/include/PrintUtils.h"

USING_NAMESPACE_QPOASES

template <typename Scalar>
TaskIkSolver<Scalar>::TaskIkSolver(
  const boost::shared_ptr<KinematicsModule<Scalar> >& km,
  const unsigned& maxIterations,
  const vector<bool>& activeJoints,
  const bool& addFinalCheck,
  const Scalar& dt,
  const Scalar& costThres,
  const Scalar& costVarThres) :
  maxIterations(maxIterations),
  dt(dt),
  costThres(costThres),
  costVarThres(costVarThres),
  activeJoints(activeJoints),
  km(km),
  addFinalCheck(addFinalCheck),
  nDof(km->getNJoints()),
  dofPLimitGain(0.5),
  maxVelocityLimitGain(0.95),
  type(JointStateType::sim),
  initiated(false)
{
  ASSERT(activeJoints.size() == nDof);
}

template <typename Scalar>
TaskIkSolver<Scalar>::~TaskIkSolver()
{
  delete qp;
  qp = NULL;
}

template <typename Scalar>
Scalar TaskIkSolver<Scalar>::computeCost()
{
  Scalar cost = 0;
  for (size_t i = 0; i < tasks.size(); ++i) {
    if (tasks[i])
      cost += tasks[i]->computeCost();
  }
  return cost;
}


template <typename Scalar>
void TaskIkSolver<Scalar>::init()
{
  //! Setup the qp solver
  qp = new SQProblem(nDof, nDof * 2);
  Options myOptions;
  myOptions.setToMPC();
  myOptions.printLevel = PL_NONE;
  qp->setOptions(myOptions);
  //! Get maximum velocity of each joint and set up velocity constraints
  //! matrices
  maxP.resize(nDof);
  minP.resize(nDof);
  maxV.resize(nDof);
  minV.resize(nDof);
  maxVIter.resize(nDof);
  minVIter.resize(nDof);
  for (size_t i = 0; i < nDof; ++i) {
    auto joint = km->getJoint(static_cast<Joints>(i));
    maxV[i] = joint->maxVelocity;
    minV[i] = -joint->maxVelocity;
    maxP[i] = joint->maxPosition;
    minP[i] = joint->minPosition;
  }
  P.resize(nDof, nDof);
  v.resize(nDof);
  G.resize(nDof * 2, nDof);
  h.resize(nDof * 2);
  G <<
    Eigen::Matrix<Scalar, Dynamic, Dynamic>::Identity(nDof, nDof),
    -Eigen::Matrix<Scalar, Dynamic, Dynamic>::Identity(nDof, nDof);
  dofPLimitGainOverdt = dofPLimitGain / dt;
  Eigen::Matrix<Scalar, Dynamic, 1> targetJoints =
    km->getJointPositions(
      Joints::first,
      toUType(Joints::count),
      JointStateType::actual);
  if (addFinalCheck) {
    finalPostureTask =
      boost::make_shared<PostureTask<Scalar> >(targetJoints, 1.0, 0.99, activeJoints, km);
    finalPostureTask->setJSType(JointStateType::sim);
  }
}

template <typename Scalar>
void TaskIkSolver<Scalar>::resetJointLimits() {
  for (size_t i = 0; i < nDof; ++i) {
    auto joint = km->getJoint(static_cast<Joints>(i));
    maxV[i] = joint->maxVelocity;
    minV[i] = -joint->maxVelocity;
    maxP[i] = joint->maxPosition;
    minP[i] = joint->minPosition;
  }
}

template <typename Scalar>
bool TaskIkSolver<Scalar>::step(
  Eigen::Matrix<Scalar, Dynamic, 1>& qd, const bool& final)
{
  ASSERT(P.rows() == P.cols() && P.rows() = nDof);
  ASSERT(v.rows() == nDof);
  P.setZero();
  v.setZero();
  //auto tStart = high_resolution_clock::now();
  if (!final) {
    for (size_t i = 0; i < tasks.size(); ++i) {
      if (tasks[i]) {
        LOG_INFO("tasks[i]: " << i)
        Eigen::Matrix<Scalar, Dynamic, Dynamic> J = tasks[i]->getJacobian();
        Eigen::Matrix<Scalar, Dynamic, Dynamic> res = tasks[i]->getGain() * tasks[i]->getResidual(dt);
        LOG_INFO("res: " << res.transpose())
        P += tasks[i]->getWeight() * J.transpose() * J;
        v += tasks[i]->getWeight() * - res.transpose() * J;
        LOG_INFO("J: " << J.transpose())
        LOG_INFO("V: " << tasks[i]->getWeight() * - res.transpose() * J)
      }
    }
  } else {
    Eigen::Matrix<Scalar, Dynamic, Dynamic> J = finalPostureTask->getJacobian();
    Eigen::Matrix<Scalar, Dynamic, Dynamic> res =
      finalPostureTask->getGain() * finalPostureTask->getResidual(dt);
    P += finalPostureTask->getWeight() * J.transpose() * J;
    v += finalPostureTask->getWeight() * - res.transpose() * J;
  }
  //duration<double> timeSpan = high_resolution_clock::now() - tStart;
  //cout << "Step 1: " << DataUtils::varToString(timeSpan.count()) << "seconds.";
  //! Set up velocity constraints matrix by choosing the minimum from
  //! maximum joint velocity and the maximum distance the joint can move
  //! due to its kinematic constraint
  /*tStart = high_resolution_clock::now();
  Eigen::Matrix<Scalar, Dynamic, Dynamic, RowMajor> Z =
    Eigen::Matrix<Scalar, Dynamic, Dynamic, RowMajor>::Zero(nDof, nDof);
  Eigen::Matrix<Scalar, Dynamic, Dynamic, RowMajor> E =
    Eigen::Matrix<Scalar, Dynamic, Dynamic, RowMajor>::Identity(nDof, nDof);
  Eigen::Matrix<Scalar, Dynamic, Dynamic, RowMajor> P0, G0;
  Eigen::Matrix<Scalar, Dynamic, 1> v0;
  Eigen::Matrix<Scalar, Dynamic, 1> h0;
  timeSpan = high_resolution_clock::now() - tStart;
  LOG_INFO("Task Ik step 3 Time: " + DataUtils::varToString(timeSpan.count()) + "seconds.");

  tStart = high_resolution_clock::now();
  P0.resize(nDof * 2, nDof * 2);
  v0.resize(nDof * 2);
  h0.resize(nDof * 3);
  G0.resize(nDof * 3, nDof * 2);
  P0.setZero();
  v0.setZero();
  G0.setZero();
  P0.block(0, 0, nDof, nDof) = P;
  P0.block(nDof, nDof, nDof, nDof) = 1e-5 * E;
  v0.segment(0, nDof) = v;
  v0.segment(nDof, nDof) = -1e-3 * Eigen::Matrix<Scalar, Dynamic, 1>::Ones(nDof);

  G0 << E, E / dt, -E, E / dt, Z, -E;
  h0 << maxV, -minV, Eigen::Matrix<Scalar, Dynamic, 1>::Zero(nDof);*/
  //tStart = high_resolution_clock::now();
  h << maxVIter, -minVIter;
  real_t* qpP = P.data();
  real_t* qpV = v.data();
  real_t* qpG = G.data();
  real_t* qpH = h.data();
  int_t nWSR = 200;
  bool solved = false;
  if (!initiated) {
    if (qp->init(qpP, qpV, qpG, NULL, NULL, NULL, qpH, nWSR, 0) == SUCCESSFUL_RETURN)
      solved = true;
    else {
      LOG_ERROR("QP failed to solve... " << final)
      //cout << "P:\n "<< P << endl;
      //cout << "v:\n  "<< v.transpose() << endl;
      //cout << "G: \n "<< G << endl;
      //cout << "h: \n "<< h << endl;
    }
    initiated = true;
  } else {
    if (qp->hotstart(qpP, qpV, qpG, NULL, NULL, NULL, qpH, nWSR, 0) == SUCCESSFUL_RETURN)
      solved = true;
    else
      LOG_ERROR("QP failed to solve... " << final)
  }
  real_t qpQd[nDof];
  qp->getPrimalSolution(qpQd);
  qd = Eigen::Map<Eigen::Matrix<Scalar, Dynamic, 1> >(qpQd, nDof, 1);
  qd[toUType(Joints::rHipYawPitch)] = qd[toUType(Joints::lHipYawPitch)];
  return solved;
}

template <typename Scalar>
Eigen::Matrix<Scalar, Dynamic, 1> TaskIkSolver<Scalar>::solve(const unsigned& maxIterations)
{
  if (addFinalCheck) {
    finalPostureTask->setActiveJoints(activeJoints);
  }
  for (size_t i = 0; i < tasks.size(); ++i) {
    if (tasks[i]) {
      tasks[i]->setJSType(type);
    }
  }
  qp->reset();
  Eigen::Matrix<Scalar, Dynamic, 1> startJoints =
    km->getJointPositions(
      Joints::first,
      toUType(Joints::count),
      JointStateType::sim);
  Eigen::Matrix<Scalar, Dynamic, 1> joints = startJoints;
  Scalar cost = 1e6, prevCost;
  for (size_t i = 0; i < maxIterations; ++i) {
    if (i > 0) {
      prevCost = cost;
      cost = computeCost();
      Scalar costVar = abs(cost - prevCost) / prevCost;
      if (abs(cost) < costThres || costVar < costVarThres) {
        break;
      }
    }
    Eigen::Matrix<Scalar, Dynamic, 1> maxDiffV = dofPLimitGainOverdt * (maxP - joints);
    Eigen::Matrix<Scalar, Dynamic, 1> minDiffV = dofPLimitGainOverdt * (minP - joints);
    maxVIter = ((maxV * maxVelocityLimitGain).array().min(maxDiffV.array())).matrix();
    minVIter = ((minV * maxVelocityLimitGain).array().max(minDiffV.array())).matrix();
    //auto tStart = high_resolution_clock::now();
    Eigen::Matrix<Scalar, Dynamic, 1> jointStep;
    bool success = step(jointStep);
    if (success) {
      jointStep *= dt;
      for (size_t j = 0; j < nDof; ++j) {
        if (activeJoints[j]) {
          joints[j] += jointStep[j];
        }
      }
      // Always update only in SIM
      km->setJointPositions(Joints::first, joints, JointStateType::sim);
    } else {
      for (size_t j = 0; j < activeJoints.size(); ++j) {
        if (activeJoints[j]) joints[j] = NAN;
      }
      break;
    }
  }
  if (addFinalCheck) {
    initiated = false;
    qp->reset();
    finalPostureTask->setTargetPosture(
      km->getJointPositions(
        Joints::first,
        toUType(Joints::count),
        JointStateType::sim));
    joints = startJoints;
    km->setJointPositions(Joints::first, startJoints, JointStateType::sim);
    Eigen::Matrix<Scalar, Dynamic, 1> maxDiffV = dofPLimitGainOverdt * (maxP - joints);
    Eigen::Matrix<Scalar, Dynamic, 1> minDiffV = dofPLimitGainOverdt * (minP - joints);
    maxVIter = ((maxV * maxVelocityLimitGain).array().min(maxDiffV.array())).matrix();
    minVIter = ((minV * maxVelocityLimitGain).array().max(minDiffV.array())).matrix();
    Eigen::Matrix<Scalar, Dynamic, 1> jointStep;
    bool success = step(jointStep, true);
    if (success) {
      jointStep *= dt;
      for (size_t j = 0; j < nDof; ++j) {
        if (activeJoints[j]) {
          joints[j] += jointStep[j];
        }
      }
      km->setJointPositions(Joints::first, joints, JointStateType::sim);
    } else {
      for (size_t j = 0; j < activeJoints.size(); ++j) {
        if (activeJoints[j]) joints[j] = NAN;
      }
    }
  }
  //Eigen::Matrix<Scalar, Dynamic, 1> aj = this->km->getJointPositions(0, toUType(Joints::count), JointStateType::actual);
  for (size_t j = 0; j < activeJoints.size(); ++j) {
    if (!activeJoints[j]) joints[j] = NAN; // || fabsf(joints[j] - aj[j]) < 1e-9
  }
  return joints;
}

template class TaskIkSolver<MType>;

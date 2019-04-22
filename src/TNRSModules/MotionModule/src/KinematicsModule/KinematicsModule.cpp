/**
 * @file MotionModule/src/KinematicsModule/KinematicsModule.cpp
 *
 * this file implements the class for solving the kinematics
 * of the robot.cout
 * Some parts of this Module have been extracted from the
 * NaoKinematics provided by:
 * @author Kofinas Nikos aka eldr4d, 2012 kouretes team
 * @author Vosk
 * @link https://github.com/kouretes/NAOKinematics
 * @cite Kofinas N., Orfanoudakis E., Lagoudakis M.: Complete Analytical
 *   Inverse Kinematics for NAO, Proceedings of the 13th International
 *   Conference on Autonomous Robot Systems and Competitions (ROBOTICA),
 *   Lisbon, Portugal, April 2013, pp. 1-6.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 08 Feb 2017
 */

#include "ControlModule/include/ActuatorRequests.h"
#include "MotionModule/include/KinematicsModule/ImuFilter.h"
#include "MotionModule/include/KinematicsModule/ComEstimator.h"
#include "MotionModule/include/KinematicsModule/ComState.h"
#include "MotionModule/include/KinematicsModule/Joint.h"
#include "MotionModule/include/KinematicsModule/JointEstimator.h"
#include "MotionModule/include/KinematicsModule/KinematicsModule.h"
#include "MotionModule/include/KinematicsModule/LinkInertiaConsts.h"
#include "MotionModule/include/KinematicsModule/LinkInfo.h"
#include "MotionModule/include/KinematicsModule/LinkChain.h"
#include "MotionModule/include/KinematicsModule/MotionTask.h"
#include "MotionModule/include/KinematicsModule/TorsoState.h"
#include "MotionModule/include/KinematicsModule/TaskIkSolver.h"
#include "MotionModule/include/MotionModule.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "Utils/include/ConfigMacros.h"
#include "Utils/include/DataUtils.h"
#include "Utils/include/Constants.h"
#include "Utils/include/JsonLogger.h"

using namespace Constants;

#ifdef ALLOW_SYMBOLIC_COMPUTATIONS
#include <symengine/matrix.h>
#include <symengine/add.h>
#include <symengine/pow.h>
#include <symengine/symengine_exception.h>
#include <symengine/visitor.h>
using SymEngine::symbol;
using SymEngine::number;
using SymEngine::DenseMatrix;
using SymEngine::Symbol;
using SymEngine::sin;
using SymEngine::cos;
using SymEngine::RCP;
using SymEngine::Basic;

template <typename Derived>
DenseMatrix eigenToSym(const MatrixBase<Derived>& eigenMat)
{
  DenseMatrix outMat = DenseMatrix(eigenMat.rows(), eigenMat.cols());
  for (size_t i = 0; i < eigenMat.rows(); ++i) {
    for (size_t j = 0; j < eigenMat.cols(); ++j) {
      outMat.set(i, j, number(eigenMat(i, j)));
    }
  }
  return outMat;
}

void expandMatrix(DenseMatrix& T)
{
  for (size_t i = 0; i < T.nrows(); ++i) {
    for (size_t j = 0; j < T.ncols(); ++j) {
      T.set(i, j, expand(T.get(i, j)));
    }
  }
}

void setIdentitySym(DenseMatrix& T)
{
  for (size_t i = 0; i < T.nrows(); ++i) {
    for (size_t j = 0; j < T.ncols(); ++j) {
      if (i == j)
        T.set(i, j, number(1));
      else
        T.set(i, j, number(0));
    }
  }
}

DenseMatrix transformVector(
  const DenseMatrix& mat,
  const DenseMatrix& vec)
{
  DenseMatrix temp =
    DenseMatrix(4, 1, {vec.get(0, 0), vec.get(1, 0), vec.get(2, 0), number(1)});
  mul_dense_dense(mat, temp, temp);
  DenseMatrix out = DenseMatrix(3, 1, {temp.get(0, 0), temp.get(1, 0), temp.get(2, 0)});
  return out;
}

#endif

#define JOINT_STATE(index, type) \
  joints[static_cast<unsigned>(index)]->states[toUType(type)]

#define JOINT_STATE_ACTUAL(index) \
  static_pointer_cast<ActualJointState<Scalar> >( \
    joints[static_cast<unsigned>(index)]->states[toUType(JointStateType::actual)])

#define JOINT_T(index, type) \
  joints[static_cast<unsigned>(index)]->states[toUType(type)]->trans

#define JOINT_T_IN_BASE(index, type) \
  joints[static_cast<unsigned>(index)]->states[toUType(type)]->transInBase

template <typename Scalar>
KinematicsModule<Scalar>::KinematicsModule(MotionModule* motionModule) :
  MemoryBase(motionModule),
  motionModule(motionModule),
  joints(toUType(Joints::count)),
  links(toUType(Links::count)), // + 1 for torso link
  footOnGround(RobotFeet::lFoot),
  ffBufferSize(15),
  globalBase(RobotFeet::lFoot),
  globalEnd(LegEEs::footCenter),
  logKinData(false)
{
  GET_CONFIG( "KinCalibration",
    (Scalar, torsoPitchOffset, torsoPitchOffset),
    (Scalar, torsoRollOffset, torsoRollOffset),
    (bool, logKinData, logKinData),
  )
  string kinDataLog = ConfigManager::getLogsDirPath() + "KinematicsModule/KinData.json";
  kinDataLogger = boost::shared_ptr<Utils::JsonLogger>(new Utils::JsonLogger(kinDataLog));
}

template <typename Scalar>
KinematicsModule<Scalar>::~KinematicsModule() {
}

template <typename Scalar>
void KinematicsModule<Scalar>::init()
{
  cycleTime = motionModule->getPeriodMinMS() / ((Scalar) 1000);
  setupLinksAndInertias();
  setupJoints();
  setupChains();
  setupWBStates();
  feetForcesBuffer.set_capacity(ffBufferSize);
  updateJointStates();
  tis = boost::shared_ptr<TaskIkSolver<Scalar> >(
          new TaskIkSolver<Scalar>(
            motionModule->getKinematicsModule(), 100, vector<bool>(), false, 2e-2)
        );
  tis->init();
  prepareDHTransforms();
  comLog.open(
    (ConfigManager::getLogsDirPath() + string("KinematicsModule/Com.txt")).c_str(),
    std::ofstream::out | std::ofstream::trunc
  );
  comLog.close();
}

template <typename Scalar>
void KinematicsModule<Scalar>::update()
{
  updateJointStates();
  prepareDHTransforms();
  updateTorsoState();
  updateComState();
  updateFootOnGround();
  updateTorsoToFeet();
  updateFootToCamT();
  //printKinematicData();
  /*static fstream zmpLog;
  Matrix<Scalar, 2, 1> zmp = computeFsrZmp(CHAIN_L_LEG);
  zmpLog.open(
    (ConfigManager::getLogsDirPath() + string("KM/Zmp.txt")).c_str(),
    std::ofstream::out | std::ofstream::app
  );
  zmpLog << zmp[0] << " " << zmp[1] << endl;
  zmpLog.close();*/
}

template <typename Scalar>
void KinematicsModule<Scalar>::cleanup()
{
  if (kinDataLogger)
    kinDataLogger->save();
}

template <typename Scalar>
void KinematicsModule<Scalar>::setupLinksAndInertias()
{
  for (size_t i = 0; i < toUType(Links::count); ++i) {
    links[i] =
      boost::shared_ptr<LinkInfo<Scalar> > (
        new LinkInfo<Scalar>()
      );
    for (size_t j = 0; j < 3; ++j) {
      for (unsigned k = 0; k < 3; ++k) {
        links[i]->inertia(j, k) = InertiaMatrices[j + i * 3][k];
      }
    }
  }
  ///< Torso mass and center of mass definitions
  links[toUType(Links::torso)]->com =
    Matrix<Scalar, 4, 1>(torsoX, torsoY, torsoZ, 1.0);
  links[toUType(Links::torso)]->mass = torsoMass;
}

template <typename Scalar>
void KinematicsModule<Scalar>::setupJoints()
{
  #ifndef V6_CROSS_BUILD
    auto& posSensors = JOINT_POSITIONS_OUT(MotionModule);
  #else
    #ifndef REALTIME_LOLA_AVAILABLE
      auto& posSensors = JOINT_POSITIONS_OUT(MotionModule);
    #else
      const auto& posSensors = JOINT_POSITIONS_IN(MotionModule);
    #endif
  #endif
  for (size_t i = 0; i < toUType(Joints::count); ++i) {
    DHParams<Scalar>* params =
      new DHParams<Scalar>(
        jointDHParams[i][0],
        jointDHParams[i][1],
        jointDHParams[i][2],
        jointDHParams[i][3]
      );
    joints[i] =
      boost::shared_ptr<Joint<Scalar> > (
        new Joint<Scalar>(
          static_cast<Joints>(i),
          jointMaxPositions[i],
          jointMinPositions[i],
          jointMaxVelocities[i],
          params,
          posSensors[i],
          this->cycleTime)
      );
    joints[i]->link = links[i];
    #ifdef ALLOW_SYMBOLIC_COMPUTATIONS
    joints[i]->symPos = symbol("q" + DataUtils::varToString(i));
    joints[i]->symVel = symbol("qd" + DataUtils::varToString(i));
    joints[i]->symAcc = symbol("qdd" + DataUtils::varToString(i));
    joints[i]->makeDHMatrixSym();
    #endif
  }
}

template <typename Scalar>
void KinematicsModule<Scalar>::setupChains()
{
  linkChains.resize(toUType(LinkChains::count));
  ///< Make head chain
  linkChains[toUType(LinkChains::head)] =
    boost::shared_ptr<LinkChain<Scalar> > (
      new LinkChain<Scalar>(LinkChains::head, HardwareIds::headStart, HardwareIds::nHead)
    );

  ///< Make larm chain
  linkChains[toUType(LinkChains::lArm)] =
    boost::shared_ptr<LinkChain<Scalar> > (
      new LinkChain<Scalar>(LinkChains::lArm, HardwareIds::lArmStart, HardwareIds::nLArm)
    );

  ///< Make rarm chain
  linkChains[toUType(LinkChains::rArm)] =
    boost::shared_ptr<LinkChain<Scalar> > (
      new LinkChain<Scalar>(LinkChains::rArm, HardwareIds::rArmStart, HardwareIds::nRArm)
    );

  ///< Make lleg chain
  linkChains[toUType(LinkChains::lLeg)] =
    boost::shared_ptr<LinkChain<Scalar> > (
      new LinkChain<Scalar>(LinkChains::lLeg, HardwareIds::lLegStart, HardwareIds::nLLeg)
    );

  ///< Make rleg chain
  linkChains[toUType(LinkChains::rLeg)] =
    boost::shared_ptr<LinkChain<Scalar> > (
      new LinkChain<Scalar>(LinkChains::rLeg, HardwareIds::rLegStart, HardwareIds::nRLeg)
    );

  for (size_t i = 0; i < toUType(LinkChains::count); ++i) {
    for (size_t j = linkChains[i]->start;
         j < linkChains[i]->start+linkChains[i]->size; ++j)
    {
      links[j]->chain = linkChains[i];
    }
  }
  ///< Set global body rotation matrix to identity()
  for (size_t i = 0; i < toUType(JointStateType::count); ++i) {
    globalToBodyRotX.push_back(Matrix<Scalar, 6, 6>::Identity());
    bodyToGlobal.push_back(Matrix<Scalar, 4, 4>::Identity());
    globalToBody.push_back(Matrix<Scalar, 4, 4>::Identity());
  }

  setupHeadChain();
  setupArmChains();
  setupLegChains();
  setupEndEffectors();

  Matrix<Scalar, 3, 3> iMat = Matrix<Scalar, 3, 3>::Identity();
  ///<Moving Inertias of all joints to to center of mass
  for (size_t i = 0; i < toUType(Links::count); ++i) {
    links[i]->inertia =
      links[i]->inertia - links[i]->mass * ((links[i]->com.segment(0, 3).transpose() * links[i]->com.segment(
        0,
        3))(0, 0) * iMat - links[i]->com.segment(0, 3) * links[i]->com.segment(0, 3).transpose());
  }

  Scalar totalChainsMass = 0;
  for (int i = 0; i < toUType(LinkChains::count); ++i) {
    linkChains[i]->mass = 0;
    for (int j = 0; j < linkChains[i]->size; ++j) {
      linkChains[i]->mass += links[linkChains[i]->start + j]->mass;
      totalChainsMass += links[linkChains[i]->start + j]->mass;
    }
  }
  LinkChain<Scalar>::totalChainsMass = totalChainsMass;

  ///< Update links partial masses used in determining com jacobian
  for (size_t i = 0; i < toUType(Joints::count); ++i) {
    joints[i]->link->partialMass = joints[i]->link->mass / totalChainsMass;
  }

  /* Inertia verification
   *
   * vector<Matrix<Scalar, 3, 1> > comsGlobal(toUType(Joints::count));
  Matrix<Scalar, 4, 4> T;
  unsigned chainStart = 0;
  unsigned n = 0;
  for (int i = 0; i < toUType(LinkChains::count); ++i) {
    T = linkChains[i]->startT;
    for (int j = 0; j < linkChains[i]->size; ++j) {
      T = T * joints[linkChains[i]->start + j]->states[ACTUAL]->T;
      Matrix<Scalar, 3, 1> com = (T * links[linkChains[i]->size + j]->com).block(0, 0, 3, 1);
      comsGlobal[chainStart + j] = com;
      ++n;
    }
    chainStart = n;
  }

  for (size_t i = 0; i < NUM_LINKS; ++i) {
    cout << "Joint[" << jointNames[i] << "]" << endl;
    cout << "com" << endl;
    cout << comsGlobal[i] << endl;
    //cout << "inertia" << endl;
    //cout <<
    //  inertiaTrans[i].transpose() *
    //  linkInertias[i] *
    //  inertiaTrans[i] << endl;
  }*/
}

template <typename Scalar>
void KinematicsModule<Scalar>::setupWBStates()
{
  torsoState =
    boost::shared_ptr<TorsoState<Scalar> > (
      new TorsoState<Scalar>()
    );
  imuFilter =
    boost::shared_ptr<ImuFilter<Scalar> > (
      new ImuFilter<Scalar>(this->cycleTime)
    );
  imuFilter->initiate();
  GET_CONFIG("KinCalibration",
    (Scalar, ImuAccelBias.x, torsoState->bias[0]),
    (Scalar, ImuAccelBias.y, torsoState->bias[1]),
    (Scalar, ImuAccelBias.z, torsoState->bias[2]),
    (Scalar, ImuAccelScale.x, torsoState->scale(0, 0)),
    (Scalar, ImuAccelScale.y, torsoState->scale(1, 1)),
    (Scalar, ImuAccelScale.z, torsoState->scale(2, 2)),
  )
  comState =
    boost::shared_ptr<ComState<Scalar> > (
      new ComState<Scalar>()
    );
  comEstimator.push_back(//X
    boost::shared_ptr<ComEstimator<Scalar, 3> > (new ComEstimator<Scalar, 3>())
  );
  comEstimator.push_back(//Y
    boost::shared_ptr<ComEstimator<Scalar, 3> > (new ComEstimator<Scalar, 3>())
  );
}

template <typename Scalar>
void KinematicsModule<Scalar>::setupHeadChain()
{
  Matrix<Scalar, 4, 4> t1;
  ///< Chain start transformation
  MathsUtils::makeTranslation(
    linkChains[toUType(LinkChains::head)]->startT,
    (Scalar) 0.0,
    (Scalar) 0.0,
    (Scalar) neckOffsetZ);
  ///< Chain end transformation
  MathsUtils::makeRotationXYZ(
    linkChains[toUType(LinkChains::head)]->endT,
    (Scalar) M_PI_2,
    (Scalar) M_PI_2,
    (Scalar) 0.0);
  ///<Masses
  links[toUType(Links::headYaw)]->mass = headYawMass;
  links[toUType(Links::headPitch)]->mass = headPitchMass;
  ///<Center of mass vectors.
  links[toUType(Links::headYaw)]->com = Matrix<Scalar, 4, 1>(headYawX, headYawY, headYawZ, 1.0f);
  links[toUType(Links::headPitch)]->com = Matrix<Scalar, 4, 1>(headPitchX, headPitchY, headPitchZ, 1.0f);

  ///<Fixing the coordinate system of center of mass.
  links[toUType(Links::headPitch)]->inertiaTrans = linkChains[toUType(LinkChains::head)]->endT.block(0, 0, 3, 3);
  links[toUType(Links::headPitch)]->com = linkChains[toUType(LinkChains::head)]->endT * links[toUType(Links::headPitch)]->com;

  ///<Transforming inertia tensor from the given frame to the joint frame.
  links[toUType(Links::headPitch)]->inertia =
    links[toUType(Links::headPitch)]->inertiaTrans *
    links[toUType(Links::headPitch)]->inertia *
    links[toUType(Links::headPitch)]->inertiaTrans.transpose();
  ///<----------------------Head End------------------------!//
}

template <typename Scalar>
void KinematicsModule<Scalar>::setupArmChains()
{
  Matrix<Scalar, 4, 4> t1;
  ///< Chain start transformation
  MathsUtils::makeTranslation(
    linkChains[toUType(LinkChains::rArm)]->startT,
    (Scalar) 0.0,
    (Scalar) -(shoulderOffsetY),
    (Scalar) shoulderOffsetZ);

  ///< Chain end transformation
  MathsUtils::makeRotationXYZ(
    linkChains[toUType(LinkChains::rArm)]->endT,
    (Scalar) -M_PI_2,
    (Scalar) 0.0,
    (Scalar) -M_PI_2);
  linkChains[toUType(LinkChains::rArm)]->endT(0, 3) = 0.f;
  linkChains[toUType(LinkChains::rArm)]->endT(1, 3) = -handOffsetZ;
  linkChains[toUType(LinkChains::rArm)]->endT(2, 3) = handOffsetX;

  ///< Masses
  links[toUType(Links::rShoulderPitch)]->mass = rShoulderPitchMass;
  links[toUType(Links::rShoulderRoll)]->mass = rShoulderRollMass;
  links[toUType(Links::rElbowYaw)]->mass = rElbowYawMass;
  links[toUType(Links::rElbowRoll)]->mass = rElbowRollMass;
  links[toUType(Links::rWristYaw)]->mass = rWristYawMass;

  ///<Center of mass vectors.
  MathsUtils::makeRotationXYZ(t1, (Scalar) M_PI_2, (Scalar) 0.0, (Scalar) 0.0);
  links[toUType(Links::rShoulderPitch)]->com = Matrix<Scalar, 4, 1>(
    rShoulderPitchX,
    rShoulderPitchY,
    rShoulderPitchZ,
    1.0f);

  ///<Fixing the coordinate system of center of mass.
  links[toUType(Links::rShoulderPitch)]->com = t1 * links[toUType(Links::rShoulderPitch)]->com;

  links[toUType(Links::rShoulderPitch)]->inertiaTrans = t1.block(0, 0, 3, 3);
  links[toUType(Links::lShoulderPitch)]->inertiaTrans = t1.block(0, 0, 3, 3);

  ///<Fixing the Inertia tensor rotation.
  links[toUType(Links::rShoulderPitch)]->inertia =
    links[toUType(Links::rShoulderPitch)]->inertiaTrans *
    links[toUType(Links::rShoulderPitch)]->inertia *
    links[toUType(Links::rShoulderPitch)]->inertiaTrans.transpose();

  /**
   * Both left and right have same transformations
   * hence Rotation Matrices
   */
  links[toUType(Links::lShoulderPitch)]->inertia =
    links[toUType(Links::lShoulderPitch)]->inertiaTrans *
    links[toUType(Links::lShoulderPitch)]->inertia *
    links[toUType(Links::lShoulderPitch)]->inertiaTrans.transpose();

  ///<Center of mass vectors.
  MathsUtils::makeRotationXYZ(t1, (Scalar) 0.0, (Scalar) 0.0, (Scalar) -M_PI_2);
  links[toUType(Links::rShoulderRoll)]->com = Matrix<Scalar, 4, 1>(
    rShoulderRollX,
    rShoulderRollY,
    rShoulderRollZ,
    1.0f);

  links[toUType(Links::rShoulderRoll)]->inertiaTrans = t1.block(0, 0, 3, 3);
  links[toUType(Links::lShoulderRoll)]->inertiaTrans = t1.block(0, 0, 3, 3);
  ///<Fixing the coordinate system of center of mass.
  links[toUType(Links::rShoulderRoll)]->com = t1 * links[toUType(Links::rShoulderRoll)]->com;

  ///<Fixing the Inertia tensor rotation.
  links[toUType(Links::rShoulderRoll)]->inertia =
    links[toUType(Links::rShoulderRoll)]->inertiaTrans *
    links[toUType(Links::rShoulderRoll)]->inertia *
    links[toUType(Links::rShoulderRoll)]->inertiaTrans.transpose();

  /**
   * Both left and right have same Scalarransformations
   * hence Rotation Matrices
   */
  links[toUType(Links::lShoulderRoll)]->inertia =
    links[toUType(Links::lShoulderRoll)]->inertiaTrans *
    links[toUType(Links::lShoulderRoll)]->inertia *
    links[toUType(Links::lShoulderRoll)]->inertiaTrans.transpose();

  ///<Center of mass vectors.
  MathsUtils::makeRotationXYZ(
    t1,
    (Scalar) -M_PI_2,
    (Scalar) 0.0,
    (Scalar) -M_PI_2);
  links[toUType(Links::rElbowYaw)]->com = Matrix<Scalar, 4, 1>(rElbowYawX, rElbowYawY, rElbowYawZ, 1.0f);


  links[toUType(Links::rElbowYaw)]->inertiaTrans = t1.block(0, 0, 3, 3);
  links[toUType(Links::lElbowYaw)]->inertiaTrans = t1.block(0, 0, 3, 3);
  ///<Fixing the coordinate system of center of mass.
  links[toUType(Links::rElbowYaw)]->com = t1 * links[toUType(Links::rElbowYaw)]->com;

  ///<Fixing the Inertia tensor rotation.
  links[toUType(Links::rElbowYaw)]->inertia =
    links[toUType(Links::rElbowYaw)]->inertiaTrans *
    links[toUType(Links::rElbowYaw)]->inertia *
    links[toUType(Links::rElbowYaw)]->inertiaTrans.transpose();

  /**
   * Both left and right have same Scalarransformations
   * hence Rotation Matrices
   */
  links[toUType(Links::lElbowYaw)]->inertia =
    links[toUType(Links::lElbowYaw)]->inertiaTrans *
    links[toUType(Links::lElbowYaw)]->inertia *
    links[toUType(Links::lElbowYaw)]->inertiaTrans.transpose();

  ///<Center of mass vectors.
  MathsUtils::makeRotationXYZ(t1, (Scalar) 0.0, (Scalar) 0.0, (Scalar) -M_PI_2);
  links[toUType(Links::rElbowRoll)]->com = Matrix<Scalar, 4, 1>(
    rElbowRollX,
    rElbowRollY,
    rElbowRollZ,
    1.0f);


  links[toUType(Links::rElbowRoll)]->inertiaTrans = t1.block(0, 0, 3, 3);
  links[toUType(Links::lElbowRoll)]->inertiaTrans = t1.block(0, 0, 3, 3);
  ///<Fixing the coordinate system of center of mass.
  links[toUType(Links::rElbowRoll)]->com = t1 * links[toUType(Links::rElbowRoll)]->com;

  ///<Fixing the Inertia tensor rotation.
  links[toUType(Links::rElbowRoll)]->inertia =
    links[toUType(Links::rElbowRoll)]->inertiaTrans *
    links[toUType(Links::rElbowRoll)]->inertia *
    links[toUType(Links::rElbowRoll)]->inertiaTrans.transpose();

  /**
   * Both left and right have same Scalarransformations
   * hence Rotation Matrices
   */
  links[toUType(Links::lElbowRoll)]->inertia =
    links[toUType(Links::lElbowRoll)]->inertiaTrans *
    links[toUType(Links::lElbowRoll)]->inertia *
    links[toUType(Links::rElbowRoll)]->inertiaTrans.transpose();

  ///<Center of mass vectors.
  MathsUtils::makeRotationXYZ(t1, (Scalar) -M_PI_2, (Scalar) 0.0, (Scalar) -M_PI_2);
  links[toUType(Links::rWristYaw)]->com = Matrix<Scalar, 4, 1>(rWristYawX, rWristYawY, rWristYawZ, 1.0f);

  links[toUType(Links::rWristYaw)]->inertiaTrans = t1.block(0, 0, 3, 3);
  links[toUType(Links::lWristYaw)]->inertiaTrans = t1.block(0, 0, 3, 3);

  ///<Fixing the coordinate system of center of mass.
  links[toUType(Links::rWristYaw)]->com = t1 * links[toUType(Links::rWristYaw)]->com;

  ///<Fixing the Inertia tensor rotation.
  links[toUType(Links::rWristYaw)]->inertia =
    links[toUType(Links::rWristYaw)]->inertiaTrans *
    links[toUType(Links::rWristYaw)]->inertia *
    links[toUType(Links::rWristYaw)]->inertiaTrans.transpose();

  /**
   * Both left and right have same Scalarransformations
   * hence Rotation Matrices
   */
  links[toUType(Links::lWristYaw)]->inertia =
    links[toUType(Links::lWristYaw)]->inertiaTrans *
    links[toUType(Links::lWristYaw)]->inertia *
    links[toUType(Links::lWristYaw)]->inertiaTrans.transpose();
  ///<-------------------Right Arm End----------------------!//

    ///<-------------------Left Arm Start--------------------!//

  ///<End and base transformations.
  MathsUtils::makeTranslation(
    linkChains[toUType(LinkChains::lArm)]->startT,
    (Scalar) 0.0,
    (Scalar) shoulderOffsetY,
    (Scalar) shoulderOffsetZ);
  linkChains[toUType(LinkChains::lArm)]->endT = linkChains[toUType(LinkChains::rArm)]->endT;

  ///<Masses and center of mass vectors
  for (size_t i = 0; i < linkChains[toUType(LinkChains::lArm)]->size; i++) {
    links[linkChains[toUType(LinkChains::lArm)]->start + i]->com =
      links[linkChains[toUType(LinkChains::rArm)]->start + i]->com;
    links[linkChains[toUType(LinkChains::lArm)]->start + i]->mass =
      links[linkChains[toUType(LinkChains::rArm)]->start + i]->mass;
  }

  ///<Fixing the center of mass coordinates
  links[toUType(Links::lShoulderPitch)]->com(2) = -links[toUType(Links::lShoulderPitch)]->com(2);
  links[toUType(Links::lShoulderRoll)]->com(0) = -links[toUType(Links::lShoulderRoll)]->com(0);
  links[toUType(Links::lElbowYaw)]->com(0) = -links[toUType(Links::lElbowYaw)]->com(0);
  links[toUType(Links::lElbowRoll)]->com(0) = -links[toUType(Links::lElbowRoll)]->com(0);
  links[toUType(Links::lWristYaw)]->com(0) = -links[toUType(Links::lWristYaw)]->com(0);

  ///<-------------------Left Arm End--------------------!//
}

template <typename Scalar>
void KinematicsModule<Scalar>::setupLegChains()
{
  Matrix<Scalar, 4, 4> t1;
  ///<------------------Right Leg Start------------------!//
  ///<End and base transformations.
  MathsUtils::makeTranslation(
    linkChains[toUType(LinkChains::rLeg)]->startT,
    (Scalar) 0.0,
    (Scalar) -hipOffsetY,
    (Scalar) -hipOffsetZ);
  MathsUtils::makeRotationZYX(
    linkChains[toUType(LinkChains::rLeg)]->endT,
    (Scalar) M_PI,
    (Scalar) -M_PI_2,
    (Scalar) 0.0);
  rotRLeg = linkChains[toUType(LinkChains::rLeg)]->endT;

  ///<Masses
  links[toUType(Joints::rHipYawPitch)]->mass = rHipYawPitchMass;
  links[toUType(Links::rHipRoll)]->mass = rHipRollMass;
  links[toUType(Links::rHipPitch)]->mass = rHipPitchMass;
  links[toUType(Links::rKneePitch)]->mass = rKneePitchMass;
  links[toUType(Links::rAnklePitch)]->mass = rAnklePitchMass;
  links[toUType(Links::rAnkleRoll)]->mass = rAnkleRollMass;

  ///<Center of mass vectors.
  MathsUtils::makeRotationXYZ(
    t1,
    (Scalar) -M_PI_2 / 2,
    (Scalar) 0.0,
    (Scalar) -M_PI_2);
  links[toUType(Joints::rHipYawPitch)]->com = Matrix<Scalar, 4, 1>(
    rHipYawPitchX,
    rHipYawPitchY,
    rHipYawPitchZ,
    1.0f);

  ///<Fixing the coordinate system of center of mass.
  t1 = MathsUtils::getTInverse(t1);

  links[toUType(Joints::rHipYawPitch)]->inertiaTrans = t1.block(0, 0, 3, 3);
  links[toUType(Joints::rHipYawPitch)]->com = t1 * links[toUType(Joints::rHipYawPitch)]->com;

  ///<Fixing the Inertia tensor rotation.
  links[toUType(Joints::rHipYawPitch)]->inertia =
    links[toUType(Joints::rHipYawPitch)]->inertiaTrans *
    links[toUType(Joints::rHipYawPitch)]->inertia *
    links[toUType(Joints::rHipYawPitch)]->inertiaTrans.transpose();

  ///<Center of mass vectors.
  MathsUtils::makeRotationXYZ(t1, (Scalar) M_PI, (Scalar) M_PI_2, (Scalar) 0.0);
  links[toUType(Links::rHipRoll)]->com = Matrix<Scalar, 4, 1>(rHipRollX, rHipRollY, rHipRollZ, 1.0f);

  links[toUType(Links::rHipRoll)]->inertiaTrans = t1.block(0, 0, 3, 3);
  ///<Fixing the coordinate system of center of mass.
  links[toUType(Links::rHipRoll)]->com = t1 * links[toUType(Links::rHipRoll)]->com;

  ///<Fixing the Inertia tensor rotation.
  links[toUType(Links::rHipRoll)]->inertia =
   links[toUType(Links::rHipRoll)]->inertiaTrans *
   links[toUType(Links::rHipRoll)]->inertia *
   links[toUType(Links::rHipRoll)]->inertiaTrans.transpose();

  ///<Center of mass vectors.
  MathsUtils::makeRotationXYZ(t1, (Scalar) M_PI_2, (Scalar) M_PI_2, (Scalar) 0.0);
  links[toUType(Links::rHipPitch)]->com = Matrix<Scalar, 4, 1>(rHipPitchX, rHipPitchY, rHipPitchZ, 1.0f);

  links[toUType(Links::rHipPitch)]->inertiaTrans = t1.block(0, 0, 3, 3);
  ///<Fixing the coordinate system of center of mass.
  links[toUType(Links::rHipPitch)]->com = t1 * links[toUType(Links::rHipPitch)]->com;

  ///<Fixing the Inertia tensor rotation.
  links[toUType(Links::rHipPitch)]->inertia =
    links[toUType(Links::rHipPitch)]->inertiaTrans *
    links[toUType(Links::rHipPitch)]->inertia *
    links[toUType(Links::rHipPitch)]->inertiaTrans.transpose();

  ///<Center of mass vectors.
  MathsUtils::makeRotationXYZ(t1, (Scalar) M_PI_2, (Scalar) M_PI_2, (Scalar) 0.0);
  links[toUType(Links::rKneePitch)]->com = Matrix<Scalar, 4, 1>(
    rKneePitchX,
    rKneePitchY,
    rKneePitchZ,
    1.0f);

  links[toUType(Links::rKneePitch)]->inertiaTrans = t1.block(0, 0, 3, 3);
  ///<Fixing the coordinate system of center of mass.
  links[toUType(Links::rKneePitch)]->com = t1 * links[toUType(Links::rKneePitch)]->com;

  ///<Fixing the Inertia tensor rotation.
  links[toUType(Links::rKneePitch)]->inertia =
    links[toUType(Links::rKneePitch)]->inertiaTrans *
    links[toUType(Links::rKneePitch)]->inertia *
    links[toUType(Links::rKneePitch)]->inertiaTrans.transpose();

  ///<Center of mass vectors.
  MathsUtils::makeRotationXYZ(t1, (Scalar) M_PI_2, (Scalar) M_PI_2, (Scalar) 0.0);
  links[toUType(Links::rAnklePitch)]->com = Matrix<Scalar, 4, 1>(
    rAnklePitchX,
    rAnklePitchY,
    rAnklePitchZ,
    1.0f);

  links[toUType(Links::rAnklePitch)]->inertiaTrans = t1.block(0, 0, 3, 3);
  ///<Fixing the coordinate system of center of mass.
  links[toUType(Links::rAnklePitch)]->com = t1 * links[toUType(Links::rAnklePitch)]->com;

  ///<Fixing the Inertia tensor rotation.
  links[toUType(Links::rAnklePitch)]->inertia =
    links[toUType(Links::rAnklePitch)]->inertiaTrans *
    links[toUType(Links::rAnklePitch)]->inertia *
    links[toUType(Links::rAnklePitch)]->inertiaTrans.transpose();

  ///<Center of mass vectors.
  links[toUType(Links::rAnkleRoll)]->com = Matrix<Scalar, 4, 1>(
    rAnkleRollX,
    rAnkleRollY,
    rAnkleRollZ,
    1.0f);

  links[toUType(Links::rAnkleRoll)]->inertiaTrans = linkChains[toUType(LinkChains::rLeg)]->endT.block(0, 0, 3, 3);
  ///<Fixing the coordinate system of center of mass.
  links[toUType(Links::rAnkleRoll)]->com = linkChains[toUType(LinkChains::rLeg)]->endT * links[toUType(Links::rAnkleRoll)]->com;

  ///<Fixing the Inertia tensor rotation.
  links[toUType(Links::rAnkleRoll)]->inertia =
    links[toUType(Links::rAnkleRoll)]->inertiaTrans *
    links[toUType(Links::rAnkleRoll)]->inertia *
    links[toUType(Links::rAnkleRoll)]->inertiaTrans.transpose();
  ///<------------------Right Leg Start------------------!//

  ///<------------------Left Leg Start-------------------!//
  ///<End and base transformations.
  MathsUtils::makeTranslation(
    linkChains[toUType(LinkChains::lLeg)]->startT,
    (Scalar) 0.0,
    (Scalar) hipOffsetY,
    (Scalar) -hipOffsetZ);
  tBaseLLegInv = linkChains[toUType(LinkChains::lLeg)]->startT;
  tBaseLLegInv = MathsUtils::getTInverse(tBaseLLegInv);
  MathsUtils::makeRotationZYX(
    linkChains[toUType(LinkChains::lLeg)]->endT,
    (Scalar) M_PI,
    (Scalar) -M_PI_2,
    (Scalar) 0.0);
  MathsUtils::makeRotationXYZ(
    rotFixLLeg,
    (Scalar) M_PI_4,
    (Scalar) 0.0,
    (Scalar) 0.0);
  tEndLLegInv = t1;
  tEndLLegInv = MathsUtils::getTInverse(tEndLLegInv);

  ///<Masses
  links[toUType(Joints::lHipYawPitch)]->mass = rHipYawPitchMass;
  links[toUType(Links::lHipRoll)]->mass = rHipRollMass;
  links[toUType(Links::lHipPitch)]->mass = rHipPitchMass;
  links[toUType(Links::lKneePitch)]->mass = rKneePitchMass;
  links[toUType(Links::lAnklePitch)]->mass = rAnklePitchMass;
  links[toUType(Links::lAnkleRoll)]->mass = rAnkleRollMass;

  ///<Center of mass vectors.
  MathsUtils::makeRotationXYZ(
    t1,
    (Scalar) -(3 * M_PI) / 4,
    (Scalar) 0.0,
    (Scalar) -M_PI_2);
  links[toUType(Joints::lHipYawPitch)]->com = Matrix<Scalar, 4, 1>(
    rHipYawPitchX,
    -rHipYawPitchY,
    rHipYawPitchZ,
    1.0f);

  ///<Fixing the coordinate system of center of mass.
  t1 = MathsUtils::getTInverse(t1);

  links[toUType(Joints::lHipYawPitch)]->inertiaTrans = t1.block(0, 0, 3, 3);
  links[toUType(Joints::lHipYawPitch)]->com = t1 * links[toUType(Joints::lHipYawPitch)]->com;

  ///<Fixing the Inertia tensor rotation.
  links[toUType(Joints::lHipYawPitch)]->inertia =
    links[toUType(Joints::lHipYawPitch)]->inertiaTrans *
    links[toUType(Joints::lHipYawPitch)]->inertia *
    links[toUType(Joints::lHipYawPitch)]->inertiaTrans.transpose();

  ///<Center of mass vectors.
  MathsUtils::makeRotationXYZ(t1, (Scalar) M_PI, (Scalar) M_PI_2, (Scalar) 0.0);
  links[toUType(Links::lHipRoll)]->com = Matrix<Scalar, 4, 1>(rHipRollX, -rHipRollY, rHipRollZ, 1.0f);

  links[toUType(Links::lHipRoll)]->inertiaTrans = t1.block(0, 0, 3, 3);
  ///<Fixing the coordinate system of center of mass.
  links[toUType(Links::lHipRoll)]->com = t1 * links[toUType(Links::lHipRoll)]->com;

  ///<Fixing the Inertia tensor rotation.
  links[toUType(Links::lHipRoll)]->inertia =
    links[toUType(Links::lHipRoll)]->inertiaTrans *
    links[toUType(Links::lHipRoll)]->inertia *
    links[toUType(Links::lHipRoll)]->inertiaTrans.transpose();

  ///<Center of mass vectors.
  MathsUtils::makeRotationXYZ(t1, (Scalar) M_PI_2, (Scalar) M_PI_2, (Scalar) 0.0);
  links[toUType(Links::lHipPitch)]->com = Matrix<Scalar, 4, 1>(rHipPitchX, -rHipPitchY, rHipPitchZ, 1.0f);

  links[toUType(Links::lHipPitch)]->inertiaTrans = t1.block(0, 0, 3, 3);
  ///<Fixing the coordinate system of center of mass.
  links[toUType(Links::lHipPitch)]->com = t1 * links[toUType(Links::lHipPitch)]->com;

  ///<Fixing the Inertia tensor rotation.
  links[toUType(Links::lHipPitch)]->inertia =
    links[toUType(Links::lHipPitch)]->inertiaTrans *
    links[toUType(Links::lHipPitch)]->inertia *
    links[toUType(Links::lHipPitch)]->inertiaTrans.transpose();

  ///<Center of mass vectors.
  MathsUtils::makeRotationXYZ(t1, (Scalar) M_PI_2, (Scalar) M_PI_2, (Scalar) 0.0);
  links[toUType(Links::lKneePitch)]->com = Matrix<Scalar, 4, 1>(
    rKneePitchX,
    -rKneePitchY,
    rKneePitchZ,
    1.0f);

  links[toUType(Links::lKneePitch)]->inertiaTrans = t1.block(0, 0, 3, 3);
  ///<Fixing the coordinate system of center of mass.
  links[toUType(Links::lKneePitch)]->com = t1 * links[toUType(Links::lKneePitch)]->com;

  ///<Fixing the Inertia tensor rotation.
  links[toUType(Links::lKneePitch)]->inertia =
    links[toUType(Links::lKneePitch)]->inertiaTrans *
    links[toUType(Links::lKneePitch)]->inertia *
    links[toUType(Links::lKneePitch)]->inertiaTrans.transpose();

  ///<Center of mass vectors.
  MathsUtils::makeRotationXYZ(t1, (Scalar) M_PI_2, (Scalar) M_PI_2, (Scalar) 0.0);
  links[toUType(Links::lAnklePitch)]->com = Matrix<Scalar, 4, 1>(
    rAnklePitchX,
    -rAnklePitchY,
    rAnklePitchZ,
    1.0f);

  links[toUType(Links::lAnklePitch)]->inertiaTrans = t1.block(0, 0, 3, 3);
  ///<Fixing the coordinate system of center of mass.
  links[toUType(Links::lAnklePitch)]->com = t1 * links[toUType(Links::lAnklePitch)]->com;

  ///<Fixing the Inertia tensor rotation.
  links[toUType(Links::lAnklePitch)]->inertia =
    links[toUType(Links::lAnklePitch)]->inertiaTrans *
    links[toUType(Links::lAnklePitch)]->inertia *
    links[toUType(Links::lAnklePitch)]->inertiaTrans.transpose();

  ///<Center of mass vectors.
  links[toUType(Links::lAnkleRoll)]->com = Matrix<Scalar, 4, 1>(
    rAnkleRollX,
    -rAnkleRollY,
    rAnkleRollZ,
    1.0f);

  links[toUType(Links::lAnkleRoll)]->inertiaTrans = linkChains[toUType(LinkChains::lLeg)]->endT.block(0, 0, 3, 3);
  ///<Fixing the coordinate system of center of mass.
  links[toUType(Links::lAnkleRoll)]->com = linkChains[toUType(LinkChains::lLeg)]->endT * links[toUType(Links::lAnkleRoll)]->com;


  ///<Fixing the Inertia tensor rotation.
  links[toUType(Links::lAnkleRoll)]->inertia =
    links[toUType(Links::lAnkleRoll)]->inertiaTrans *
    links[toUType(Links::lAnkleRoll)]->inertia *
    links[toUType(Links::lAnkleRoll)]->inertiaTrans.transpose();

  ///<------------------Left Leg End-------------------!//
}

template <typename Scalar>
void KinematicsModule<Scalar>::setupEndEffectors()
{
  Matrix<Scalar, 4, 4> t1;
  linkChains[toUType(LinkChains::head)]->endEffectors.resize(toUType(CameraId::count));
  linkChains[toUType(LinkChains::lLeg)]->endEffectors.resize(toUType(LegEEs::count));
  linkChains[toUType(LinkChains::rLeg)]->endEffectors.resize(toUType(LegEEs::count));
  linkChains[toUType(LinkChains::lArm)]->endEffectors.push_back(Matrix<Scalar, 4, 4>::Identity());
  linkChains[toUType(LinkChains::rArm)]->endEffectors.push_back(Matrix<Scalar, 4, 4>::Identity());
  MathsUtils::makeTranslation(
    linkChains[toUType(LinkChains::head)]->endEffectors[toUType(CameraId::headTop)],
    (Scalar) cameraTopX,
    (Scalar) 0.0,
    (Scalar) cameraTopZ);
  MathsUtils::makeTranslation(
    linkChains[toUType(LinkChains::head)]->endEffectors[toUType(CameraId::headBottom)],
    (Scalar) cameraBottomX,
    (Scalar) 0.0,
    (Scalar) cameraBottomZ);
  MathsUtils::makeRotationXYZ(t1, (Scalar) 0.0, (Scalar) M_PI_2, (Scalar) -M_PI_2);
  linkChains[toUType(LinkChains::head)]->endEffectors[toUType(CameraId::headTop)] =
    linkChains[toUType(LinkChains::head)]->endEffectors[toUType(CameraId::headTop)] * t1;
  linkChains[toUType(LinkChains::head)]->endEffectors[toUType(CameraId::headBottom)] =
    linkChains[toUType(LinkChains::head)]->endEffectors[toUType(CameraId::headBottom)] * t1;
  MathsUtils::makeRotationXYZ(t1, (Scalar) -cameraTopAngleY, //Y becomes -X after frame transfomration
    (Scalar) 0.0,
    (Scalar) 0.0);
  linkChains[toUType(LinkChains::head)]->endEffectors[toUType(CameraId::headTop)] =
    linkChains[toUType(LinkChains::head)]->endEffectors[toUType(CameraId::headTop)] * t1;
  MathsUtils::makeRotationXYZ(t1, (Scalar) -cameraBotAngleY, //Y becomes -X after frame transfomration
    (Scalar) 0.0,
    (Scalar) 0.0);
  linkChains[toUType(LinkChains::head)]->endEffectors[toUType(CameraId::headBottom)] =
    linkChains[toUType(LinkChains::head)]->endEffectors[toUType(CameraId::headBottom)] * t1;

  //cout << linkChains[toUType(LinkChains::head)]->endEffectors[toUType(CameraId::headTop)] << endl;
  //cout << linkChains[toUType(LinkChains::head)]->endEffectors[toUType(CameraId::headBottom)] << endl;

  linkChains[toUType(LinkChains::lLeg)]->endEffectors[toUType(LegEEs::ankle)] = Matrix<Scalar, 4, 4>::Identity();
  linkChains[toUType(LinkChains::rLeg)]->endEffectors[toUType(LegEEs::ankle)] = Matrix<Scalar, 4, 4>::Identity();
  linkChains[toUType(LinkChains::rLeg)]->endEffectors[toUType(LegEEs::footLeftBound)] = Matrix<Scalar, 4, 4>::Identity();
  linkChains[toUType(LinkChains::rLeg)]->endEffectors[toUType(LegEEs::footRightBound)] = Matrix<Scalar, 4, 4>::Identity();

  MathsUtils::makeTranslation(
    linkChains[toUType(LinkChains::lLeg)]->endEffectors[toUType(LegEEs::footBase)],
    (Scalar) 0.0,
    (Scalar) 0.0,
    (Scalar) -footHeight);
  linkChains[toUType(LinkChains::rLeg)]->endEffectors[toUType(LegEEs::footBase)] =
    linkChains[toUType(LinkChains::lLeg)]->endEffectors[toUType(LegEEs::footBase)];

  MathsUtils::makeTranslation(
    linkChains[toUType(LinkChains::lLeg)]->endEffectors[toUType(LegEEs::footCenter)],
    (Scalar) Constants::footOriginShiftX,
    (Scalar) Constants::footOriginShiftY,
    (Scalar) -footHeight);
  linkChains[toUType(LinkChains::rLeg)]->endEffectors[toUType(LegEEs::footCenter)] =
    linkChains[toUType(LinkChains::lLeg)]->endEffectors[toUType(LegEEs::footCenter)];
  linkChains[toUType(LinkChains::rLeg)]->endEffectors[toUType(LegEEs::footCenter)](1, 3) *= -1;

  //cout << "footcenterLleg:" << linkChains[toUType(LinkChains::lLeg)]->endEffectors[toUType(LegEEs::footCenter)] << endl;
  //cout << "footcenterRleg:" << linkChains[toUType(LinkChains::rLeg)]->endEffectors[toUType(LegEEs::footCenter)] << endl;

  MathsUtils::makeTranslation(
    linkChains[toUType(LinkChains::lLeg)]->endEffectors[toUType(LegEEs::footLeftBound)],
    (Scalar) footSizeX / 2 + footOriginShiftX,
    (Scalar) footSizeY / 2,
    (Scalar) -footHeight);
  linkChains[toUType(LinkChains::rLeg)]->endEffectors[toUType(LegEEs::footLeftBound)] = linkChains[toUType(LinkChains::lLeg)]->endEffectors[toUType(LegEEs::footLeftBound)];

  MathsUtils::makeTranslation(
    linkChains[toUType(LinkChains::lLeg)]->endEffectors[toUType(LegEEs::footRightBound)],
    (Scalar) footSizeX / 2 + footOriginShiftX,
    (Scalar) -footSizeY / 2,
    (Scalar) -footHeight);
  linkChains[toUType(LinkChains::rLeg)]->endEffectors[toUType(LegEEs::footRightBound)] = linkChains[toUType(LinkChains::lLeg)]->endEffectors[toUType(LegEEs::footRightBound)];

  //LOG_INFO("L_toUType(LegEEs::footRightBound):\n" <<linkChains[toUType(LinkChains::lLeg)]->endEffectors[toUType(LegEEs::footRightBound)])
  //LOG_INFO("L_toUType(LegEEs::footRightBound):\n" <<linkChains[toUType(LinkChains::lLeg)]->endEffectors[toUType(LegEEs::footLeftBound)])
}

template <typename Scalar>
void KinematicsModule<Scalar>::printKinematicData()
{
  IOFormat OctaveFmt(StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
  cout << "-------------------________________Printing Kinematic Data________________-------------------" << endl;
  for (int i = 0; i < toUType(Joints::count); ++i) {
    cout << "Joint[" << joints[i]->name << "]" << endl;
    cout << "Position:" << JOINT_STATE(i, JointStateType::actual)->position() * 180 / M_PI << ",  ";
    //cout << "Velocity:" << JOINT_STATE(i, JointStateType::actual)->velocity() << ",  ";
    //cout << "Acceleration:" << JOINT_STATE(i, JointStateType::actual)->accel() << endl;
    //cout << "Com: [" << links[i]->com.format(OctaveFmt) << " ,  Mass:" << links[i]->mass << endl;
    //cout << "Inertias: " << endl << links[i]->inertia.format(OctaveFmt) << endl << " ------------------------ " << endl;
    //cout << "DHMatrix: " << endl << JOINT_STATE(i, JointStateType::actual)->trans << endl << " ------------------------ " << endl;
  }
  //cout << "[Torso]" << endl;
  //cout << "Com: [" << links[toUType(Joints::count)]->com.format(OctaveFmt) << " ,  Mass:" << links[toUType(Joints::count)]->mass << endl;
  //cout << "Inertia: " << endl << links[toUType(Joints::count)]->inertia.format(OctaveFmt) << endl;
}

template <typename Scalar>
void KinematicsModule<Scalar>::updateJointStates()
{
  try {
    #ifndef V6_CROSS_BUILD
      auto& posSensors = JOINT_POSITIONS_OUT(MotionModule);
    #else
      #ifndef REALTIME_LOLA_AVAILABLE
        auto& posSensors = JOINT_POSITIONS_OUT(MotionModule);
      #else
        const auto& posSensors = JOINT_POSITIONS_IN(MotionModule);
      #endif
    #endif
    for (size_t i = 0; i < toUType(Joints::count); ++i) {
      JOINT_STATE_ACTUAL(i)->update(posSensors[i]);
    }
  } catch (const exception& e) {
    LOG_EXCEPTION(e.what());
  }
}

/**
 * Inertial sensors enum defined in Utils/include/HardwareIds.h
  GYROSCOPE_X = 0,
  GYROSCOPE_Y,
  GYROSCOPE_Z,
  TORSO_ANGLE_X,
  TORSO_ANGLE_Y,
  TORSO_ANGLE_Z,
  ACCELEROMETER_X,
  ACCELEROMETER_Y,
  ACCELEROMETER_Z,
 */

template <typename Scalar>
void KinematicsModule<Scalar>::updateTorsoState()
{
  try {
    // Need a kalman filter for this tracker.
    // Nao sensors are updated almost with cycle time of 40ms in memory
    // Whereas motion module runs at 10ms
    #ifndef V6_CROSS_BUILD
      auto& inertial = INERTIAL_SENSORS_OUT(MotionModule);
    #else
      #ifndef REALTIME_LOLA_AVAILABLE
        auto& inertial = INERTIAL_SENSORS_OUT(MotionModule);
      #else
        const auto& inertial = INERTIAL_SENSORS_IN(MotionModule);
      #endif
    #endif
    static auto gravity = Matrix<Scalar, 3, 1>(0.0, 0.0, -Constants::gravity);
    torsoState->accel[0] = inertial[toUType(InertialSensors::accelerometerX)];
    torsoState->accel[1] = inertial[toUType(InertialSensors::accelerometerY)];
    torsoState->accel[2] = inertial[toUType(InertialSensors::accelerometerZ)];
    torsoState->accel = torsoState->scale * (torsoState->accel - torsoState->bias);
    torsoState->angularVelocity[0] = inertial[toUType(InertialSensors::gyroscopeX)];
    torsoState->angularVelocity[1] = inertial[toUType(InertialSensors::gyroscopeY)];
    torsoState->angularVelocity[2] = 0.0; // No data
    // Madgewick filter not giving right orientation...
    // imuFilter->update(torsoState->accel, torsoState->angularVelocity);
    // torsoState->rot.block(0, 0, 3, 3) = imuFilter->getRotation();
    // LOG_INFO("rotated accel: " << torsoState->rot.block(0, 0, 3, 3) * torsoState->accel);
    // torsoState->accel = torsoState->rot.block(0, 0, 3, 3) * torsoState->accel - gravity;
    //LOG_INFO("accel: " << torsoState->rot.block(0, 0, 3, 3) * torsoState->accel - gravity);
    MathsUtils::makeRotationXYZ(
      torsoState->rot,
      (Scalar) inertial[toUType(InertialSensors::torsoAngleX)],
      (Scalar) inertial[toUType(InertialSensors::torsoAngleY)],
      0.0);
    //LOG_INFO("accel: " << torsoState->rot.block(0, 0, 3, 3) * torsoState->accel);
    torsoState->accel = torsoState->rot.block(0, 0, 3, 3) * torsoState->accel - gravity;
    torsoState->velocity = torsoState->velocity - torsoState->accel * cycleTime;
    if(logKinData) {
      /*Json::Value angle;
      angle.append(inertial[toUType(InertialSensors::torsoAngleX)]);
      angle.append(inertial[toUType(InertialSensors::torsoAngleY)]);
      Json::Value accel;
      accel.append(inertial[toUType(InertialSensors::accelerometerX)]);
      accel.append(inertial[toUType(InertialSensors::accelerometerY)]);
      accel.append(inertial[toUType(InertialSensors::accelerometerZ)]);
      JSON_APPEND(kinDataLogger->getRoot(), "imuAngleMeas", angle);
      JSON_APPEND(kinDataLogger->getRoot(), "imuAccelMeas", accel);*/
    }
  } catch (const exception& e) {
    LOG_EXCEPTION(e.what());
  }
}

template <typename Scalar>
void KinematicsModule<Scalar>::updateComState()
{
  Matrix<Scalar, 3, 1> comFoot;
  Matrix<Scalar, 2, 1> fsrZmp = computeFsrZmp(globalBase);
  //computeComWrtBase(globalBase, globalEnd, comFoot, JointStateType::sim);
  computeComWrtBase(static_cast<LinkChains>(globalBase), toUType(globalEnd), comFoot, JointStateType::actual);
  comState->position[2] = (Scalar) comFoot[2];
  Matrix<Scalar, 3, 1> imuPos =
    (globalToBody[(unsigned)JointStateType::actual] * Matrix<Scalar, 4, 1>(-0.008, 0.00606, 0.027, 1.0)).block(0, 0, 3, 1);
  //LOG_INFO("imuPos:" << imuPos)
  Matrix<Scalar, 3, 1> imuToCom = comFoot - imuPos;
  //LOG_INFO("imuToCom:" << imuToCom)
  Matrix<Scalar, 3, 1> comAccel =
    torsoState->accel + torsoState->angularVelocity.cross(torsoState->angularVelocity.cross(imuToCom));
  for (int i = 0; i < comEstimator.size(); ++i) {
    if (!comEstimator[i]->isInitiated()) {
      Matrix<Scalar, Dynamic, 1> state(3, 1);
      state << comFoot[i], 0.0, 0.0;
      comEstimator[i]->init(state, comFoot[2], cycleTime);
    } else {
      comEstimator[i]->updateComHeight(comFoot[2]);
      Matrix<Scalar, Dynamic, 1> meas(3, 1);
      meas << comFoot[i], -comAccel[i], fsrZmp[i];
      //LOG_INFO("meas: " << meas.transpose())
      comEstimator[i]->update(meas);
      Matrix<Scalar, Dynamic, 1> est = comEstimator[i]->getState();
      comState->position[i] = est[0];
      comState->velocity[i] = est[1];
      comState->accel[i] = est[2];
      comState->zmp[i] = comEstimator[i]->getOutput()[0];
    }
  }
  comLog.open(
    (ConfigManager::getLogsDirPath() + string("KinematicsModule/Com.txt")).c_str(),
    std::ofstream::out | std::ofstream::app
  );
  comLog << motionModule->getModuleTime() << " "
         << comState->position[0] << " "
         << comState->position[1] << " "
         << comFoot[0] << " "
         << comFoot[1] << " "
         << comState->velocity[0] << " "
         << comState->velocity[1] << " "
         << comState->accel[0] << " "
         << comState->accel[1] << " "
         << torsoState->accel[0] << " "
         << torsoState->accel[1] << " "
         << comState->zmp[0] << " "
         << comState->zmp[1] << " "
         << fsrZmp[0] << " "
         << fsrZmp[1] << " "
         << comAccel[0] << " "
         << comAccel[1] << endl;
  comLog.close();
  if (logKinData) {
    Json::Value comMeas;
    comMeas.append(comFoot[0]);
    comMeas.append(comFoot[1]);
    JSON_APPEND(kinDataLogger->getRoot(), "comMeas", comMeas);
  }
}

template <typename Scalar>
void KinematicsModule<Scalar>::prepareDHTransforms(
  const LinkChains& ch, const JointStateType& type, const bool& solveFk)
{
  if (solveFk) {
    if (ch == LinkChains::count) {
      for (size_t c = 0; c < toUType(ch); ++c) {
        Matrix<Scalar, 4, 4> T = linkChains[c]->startT; // First transform
        for (size_t i = linkChains[c]->start; i < linkChains[c]->start + linkChains[c]->size; ++i) {
          T = T * joints[i]->computeLinkTrans(type);
          // transformation from base torso
          joints[i]->setTransInBase(T, type);
        }
        linkChains[c]->jacobianInfo[toUType(type)]->reset();
        if (c == toUType(globalBase)) {
          bodyToGlobal[toUType(type)] = getForwardEffector(static_cast<LinkChains>(globalBase), toUType(globalEnd), type);
          globalToBody[toUType(type)] = MathsUtils::getTInverse(bodyToGlobal[toUType(type)]);
          Matrix<Scalar, 3, 3> rot = globalToBody[toUType(type)].block(0, 0, 3, 3);
          globalToBodyRotX[toUType(type)].block(0, 0, 3, 3) = rot;
          globalToBodyRotX[toUType(type)].block(3, 3, 3, 3) = rot;
        }
      }
    } else {
      Matrix<Scalar, 4, 4> T = linkChains[toUType(ch)]->startT; // First transform
      for (size_t i = linkChains[toUType(ch)]->start; i < linkChains[toUType(ch)]->start + linkChains[toUType(ch)]->size; ++i) {
        T = T * joints[i]->computeLinkTrans(type);
        // transformation from base torso
        joints[i]->setTransInBase(T, type);
      }
      linkChains[toUType(ch)]->jacobianInfo[toUType(type)]->reset();
      if (ch == static_cast<LinkChains>(globalBase)) {
        bodyToGlobal[toUType(type)] = getForwardEffector(static_cast<LinkChains>(globalBase), toUType(globalEnd), type);
        globalToBody[toUType(type)] = MathsUtils::getTInverse(bodyToGlobal[toUType(type)]);
        Matrix<Scalar, 3, 3> rot = globalToBody[toUType(type)].block(0, 0, 3, 3);
        globalToBodyRotX[toUType(type)].block(0, 0, 3, 3) = rot;
        globalToBodyRotX[toUType(type)].block(3, 3, 3, 3) = rot;
      }
    }
  } else {
    if (ch == LinkChains::count) {
      for (size_t c = 0; c < toUType(ch); ++c) {
        for (size_t i = linkChains[c]->start; i < linkChains[c]->start + linkChains[c]->size; ++i) {
          joints[i]->computeLinkTrans(type);
        }
      }
    } else {
      for (size_t i = linkChains[toUType(ch)]->start; i < linkChains[toUType(ch)]->start + linkChains[toUType(ch)]->size; ++i) {
        joints[i]->computeLinkTrans(type);
      }
    }
  }
}

template <typename Scalar>
void KinematicsModule<Scalar>::setEndEffector(const LinkChains& chain, const unsigned& eeIndex,
  const Matrix<Scalar, 4, 1>& ee)
{
  Matrix<Scalar, 4, 4> t1;
  if (chain == LinkChains::rLeg) {
    MathsUtils::makeTranslation(t1, ee[0], ee[1], ee[2]);
    linkChains[toUType(chain)]->endEffectors[eeIndex] = t1;
    MathsUtils::makeTranslation(t1, ee[0], -ee[1], ee[2]);
    tEndLLegInv = t1;
    tEndLLegInv = MathsUtils::getTInverse(tEndLLegInv);
  } else if (chain == LinkChains::lLeg) {
    MathsUtils::makeTranslation(t1, ee[0], ee[1], ee[2]);
    linkChains[toUType(chain)]->endEffectors[eeIndex] = t1;
    tEndLLegInv = t1;
    tEndLLegInv = MathsUtils::getTInverse(tEndLLegInv);
  } else {
    MathsUtils::makeTranslation(t1, ee[0], ee[1], ee[2]);
    linkChains[toUType(chain)]->endEffectors[eeIndex] = t1;
  }
}

#ifdef ALLOW_SYMBOLIC_COMPUTATIONS
template <typename Scalar>
SymEngine::DenseMatrix KinematicsModule<Scalar>::getForwardEffectorSym(
  const unsigned& chainIndex, const Matrix<Scalar, 4, 4> &endEffector)
{
  unsigned start = linkChains[chainIndex]->start;
  unsigned size = linkChains[chainIndex]->size;
  DenseMatrix T = eigenToSym(linkChains[chainIndex]->startT); // First transform

  for (size_t i = start; i < start + size; ++i) {
    mul_dense_dense(T, joints[i]->symTrans, T);
    expandMatrix(T);
  }
  mul_dense_dense(T, eigenToSym(linkChains[chainIndex]->endT),  T);
  mul_dense_dense(T, eigenToSym(endEffector),  T);
  return T;
}

template <typename Scalar>
SymEngine::DenseMatrix KinematicsModule<Scalar>::getForwardEffectorSym(
  const unsigned& chainIndex, const unsigned& eeIndex)
{
  return getForwardEffectorSym(chainIndex, linkChains[chainIndex]->endEffectors[eeIndex]);
}
#endif

template <typename Scalar>
Matrix<Scalar, 4, 4>
KinematicsModule<Scalar>::getForwardEffector(
  const LinkChains& chainIndex, const Matrix<Scalar, 4, 4> &endEffector, const JointStateType& type)
{
  return JOINT_T_IN_BASE(
    linkChains[toUType(chainIndex)]->start + linkChains[toUType(chainIndex)]->size - 1, type) *
    linkChains[toUType(chainIndex)]->endT *
    endEffector;
}

template <typename Scalar>
Matrix<Scalar, 4, 4>
KinematicsModule<Scalar>::getForwardEffector(
  const LinkChains& chainIndex, const unsigned& eeIndex, const JointStateType& type)
{
  return getForwardEffector(chainIndex, linkChains[toUType(chainIndex)]->endEffectors[eeIndex], type);
}

template <typename Scalar>
ComState<Scalar>
KinematicsModule<Scalar>::getComStateWrtFrame(
  const LinkChains& baseFrame,
  const unsigned& eeIndex)
{
  ComState<Scalar> tComState = *comState; // Get current com state
  // These readings will also require a kalman filter for good com state
  if (baseFrame == static_cast<LinkChains>(tComState.baseFrame)) {
    return tComState;
  } else if (toUType(baseFrame) >= 0 && toUType(baseFrame) < toUType(LinkChains::count)) {
    Matrix<Scalar, 4, 4> T =
      MathsUtils::getTInverse(getForwardEffector(baseFrame, eeIndex)) *
      getForwardEffector(static_cast<LinkChains>(tComState.baseFrame), tComState.eeIndex);
    tComState.position = MathsUtils::transformVector(T, tComState.position);
    tComState.velocity = T.block(0, 0, 3, 3) * tComState.velocity;
    tComState.accel = T.block(0, 0, 3, 3) * tComState.accel;
  }
  return tComState;
}

template <typename Scalar>
boost::shared_ptr<ProcessModel<Scalar, 3, 1, 1> >
KinematicsModule<Scalar>::getComModel(const unsigned& index) {
  return comEstimator[index]->getModel();
}

template <typename Scalar>
void
KinematicsModule<Scalar>::computeComWrtBase(const LinkChains& limbIndex,
  const unsigned& eeIndex, Matrix<Scalar, 3, 1> &comVector, const JointStateType& type)
{
  Matrix<Scalar, 3, 1> comWrtTorso = calculateCenterOfMass(type);
  if (limbIndex != LinkChains::count) {
    comVector = MathsUtils::transformVector(
      MathsUtils::getTInverse(getForwardEffector(limbIndex, eeIndex, type)),
      comWrtTorso);
  } else {
    comVector = comWrtTorso;
  }
}

template <typename Scalar>
void
KinematicsModule<Scalar>::computeComWrtBase(const LinkChains& limbIndex,
  const unsigned& eeIndex, Matrix<Scalar, 2, 1>& comVector, const JointStateType& type)
{
  Matrix<Scalar, 3, 1> com;
  computeComWrtBase(limbIndex, eeIndex, com, type);
  comVector[0] = com(0, 0);
  comVector[1] = com(1, 0);
}

template <typename Scalar>
Matrix<Scalar, 3, 1>
KinematicsModule<Scalar>::computeComWrtBase(const LinkChains& limbIndex,
  const unsigned& eeIndex, const JointStateType& type)
{
  if (limbIndex != LinkChains::count) {
    Matrix<Scalar, 3, 1> com =
      MathsUtils::transformVector(
        MathsUtils::getTInverse(getForwardEffector(limbIndex, eeIndex, type)),
        calculateCenterOfMass(type));
    return com;
  } else {
    return calculateCenterOfMass(type);
  }
}

template <typename Scalar>
Matrix<Scalar, Dynamic, 1>
KinematicsModule<Scalar>::cartToJointVels(
  const LinkChains& chainIndex,
  const Matrix<Scalar, Dynamic, 1> cVels,
  const Matrix<Scalar, 4, 4> endEffector,
  const JointStateType& type)
{
  Matrix<Scalar, Dynamic, Dynamic> jacobian = computeLimbJ(chainIndex, endEffector, type, false);
  return MathsUtils::pseudoInverseSolve(jacobian, cVels);
}

template <typename Scalar>
Matrix<Scalar, 6, Dynamic>
KinematicsModule<Scalar>::computeLimbJ(
  const LinkChains& chainIndex,
  const unsigned& eeIndex,
  const JointStateType& type,
  const bool& relGlobalBase)
{
  Matrix<Scalar, 6, Dynamic> jacobian;
  if (linkChains[toUType(chainIndex)]->jacobianInfo[toUType(type)]->getJacobian(jacobian, eeIndex)) {
    return jacobian;
  }
  unsigned size = linkChains[toUType(chainIndex)]->size;
  unsigned chainStart = linkChains[toUType(chainIndex)]->start;
  Matrix<Scalar, 4, 4> endEffector = linkChains[toUType(chainIndex)]->endEffectors[eeIndex];
  Matrix<Scalar, 3, Dynamic> jacobianV;
  Matrix<Scalar, 3, Dynamic> jacobianW;
  jacobianV.resize(3, size);
  jacobianW.resize(3, size);
  Matrix<Scalar, 3, 1> eePos =
    getForwardEffector(chainIndex, endEffector, type).template block<3, 1>(0, 3);
  for (size_t i = 0; i < size; ++i) {
    jacobianV.col(i) =
      JOINT_STATE(chainStart+i, type)->zInBase.cross(
        eePos - JOINT_STATE(chainStart+i, type)->posInBase
      );
    jacobianW.col(i) = JOINT_STATE(chainStart+i, type)->zInBase;
  }
  jacobian.resize(6, size);
  jacobian << jacobianV, jacobianW;
  if (relGlobalBase) {
    ///< Get body center frame rotation in support foot frame
    jacobian = globalToBodyRotX[toUType(type)] * jacobian;
    linkChains[toUType(chainIndex)]->jacobianInfo[toUType(type)]->setJacobian(jacobian, eeIndex);
  }
  return jacobian;
}

template <typename Scalar>
Matrix<Scalar, 6, Dynamic>
KinematicsModule<Scalar>::computeGlobalBaseJ(
  const JointStateType& type)
{
  return computeLimbJ(static_cast<LinkChains>(globalBase), toUType(globalEnd), type);
}

template <typename Scalar>
Matrix<Scalar, 6, Dynamic>
KinematicsModule<Scalar>::computeLimbJ(
  const LinkChains& chainIndex,
  const Matrix<Scalar, 4, 4>& endEffector,
  const JointStateType& type,
  const bool& relGlobalBase)
{
  Matrix<Scalar, 6, Dynamic> jacobian;
  unsigned size = linkChains[toUType(chainIndex)]->size;
  unsigned chainStart = linkChains[toUType(chainIndex)]->start;
  Matrix<Scalar, 3, Dynamic> jacobianV;
  Matrix<Scalar, 3, Dynamic> jacobianW;
  jacobianV.resize(3, size);
  jacobianW.resize(3, size);
  Matrix<Scalar, 3, 1> eePos =
    getForwardEffector(chainIndex, endEffector, type).template block<3, 1>(0, 3);
  for (size_t i = 0; i < size; ++i) {
    jacobianV.col(i) =
      JOINT_STATE(chainStart+i, type)->zInBase.cross(
        eePos - JOINT_STATE(chainStart+i, type)->posInBase
      );
    jacobianW.col(i) = JOINT_STATE(chainStart+i, type)->zInBase;
  }
  jacobian.resize(6, size);
  jacobian << jacobianV, jacobianW;
  if (relGlobalBase) {
    ///< Get body center frame rotation in support foot frame
    jacobian = globalToBodyRotX[toUType(type)] * jacobian;
  }
  return jacobian;
}

template <typename Scalar>
Matrix<Scalar, 3, Dynamic>
KinematicsModule<Scalar>::computeComJacobian(const JointStateType& type)
{
  Matrix<Scalar, 3, Dynamic> jacobian;//, jv, jw;
  jacobian.resize(3, toUType(Joints::count));
  ///< Get the center of mass jacobian in body center frame
  for (size_t i = 0; i < toUType(LinkChains::count); ++i) { // FIXME LATER CHANGING i=0 to i=3 for testing computation speed
    jacobian.block(0, linkChains[i]->start, 3, linkChains[i]->size) =
      computeLimbComJ(static_cast<LinkChains>(i), type);
  }
  ///< Convert the center of mass jacobian in global base frame
  jacobian = globalToBody[unsigned(type)].block(0, 0, 3, 3) * jacobian;

  Matrix<Scalar, 6, Dynamic> baseJ = computeGlobalBaseJ(type);
  Matrix<Scalar, 3, 1> com = computeComWrtBase(static_cast<LinkChains>(globalBase), toUType(globalEnd), type);
  auto baseStart = linkChains[toUType(globalBase)]->start;
  auto baseSize = linkChains[toUType(globalBase)]->size;
  jacobian.block(0, baseStart, 3, baseSize) =
    jacobian.block(0, baseStart, 3, baseSize) -
    baseJ.block(0, 0, 3, baseSize) +
    MathsUtils::makeSkewMat(com) * baseJ.block(3, 0, 3, baseSize);
  return jacobian;
}

template <typename Scalar>
Matrix<Scalar, 3, Dynamic>
KinematicsModule<Scalar>::computeLimbComJ(
  const LinkChains& chainIndex,
  const JointStateType& type)
{
  unsigned size = linkChains[toUType(chainIndex)]->size;
  unsigned chainStart = linkChains[toUType(chainIndex)]->start;
  Matrix<Scalar, 3, Dynamic> jacobian(3, size);
  jacobian.setZero();
  /*for (int j = 0; j < size; ++j) {
    Matrix<Scalar, 3, 1> z = JOINT_STATE(chainStart + j, type)->zInBase;
    Matrix<Scalar, 3, 1> pos = JOINT_STATE(chainStart + j, type)->posInBase;
    for (int m = j; m < size; ++m) { // Update the jth row of the jacobian
      jacobian.col(j) +=
        z.cross(JOINT_STATE(chainStart + m, type)->comInBase - pos) *
        joints[chainStart + m]->link->mass;/// joints[chainStart + m]->link->chain->mass;
    }
  }*/
  Matrix<Scalar, 3, Dynamic> jLink;
  jLink.resize(3, size);
  jLink.setZero();
  //Scalar chainMass = joints[chainStart]->link->chain->mass;
  for (int j = 0; j < size; ++j) {
    Matrix<Scalar, 3, 1> comT = JOINT_STATE(chainStart + j, type)->comInBase;
    for (int m = 0; m <= j; ++m) { // Update the jth row of the jacobian
      Matrix<Scalar, 3, 1> z = JOINT_STATE(chainStart + m, type)->zInBase;
      Matrix<Scalar, 3, 1> pos = JOINT_STATE(chainStart + m, type)->posInBase;
      jLink.col(m) += z.cross(comT - pos);
    }
    jacobian += jLink * joints[chainStart + j]->link->mass;
  }
  jacobian /= totalMassH25;
  return jacobian;
}

template <typename Scalar>
void
KinematicsModule<Scalar>::computeLinkComJ(
  const Links& index,
  Matrix<Scalar, 3, Dynamic>& jacobianV,
  Matrix<Scalar, 3, Dynamic>& jacobianW,
  const JointStateType& type)
{
  unsigned chainStart = joints[toUType(index)]->link->chain->start;
  unsigned size = joints[toUType(index)]->link->chain->size;
  jacobianV.resize(3, size);
  jacobianW.resize(3, size);
  jacobianV.setZero();
  jacobianW.setZero();
  Matrix<Scalar, 3, 1> comT = JOINT_STATE(index, type)->comInBase;
  for (int m = 0; m <= toUType(index) - chainStart; ++m) { // Update the jth row of the jacobian
    Matrix<Scalar, 3, 1> z = JOINT_STATE(chainStart + m, type)->zInBase;
    Matrix<Scalar, 3, 1> pos = JOINT_STATE(chainStart + m, type)->posInBase;
    jacobianV.col(m) = z.cross(comT - pos);
    jacobianW.col(m) = z;
  }
}

template <typename Scalar>
Matrix<Scalar, Dynamic, 1>
KinematicsModule<Scalar>::solveComIkTwoVar(
  const Matrix<Scalar, 3, 1>& desCom,
  const LinkChains& baseLimb,
  const unsigned& eeIndex,
  const JointStateType& type)
{
  Matrix<Scalar, 3, 1> com;
  computeComWrtBase(baseLimb, eeIndex, com, type);
  Matrix<Scalar, 3, 1> diff = desCom - com;
  Scalar aY = atan2(diff[1], com[2]);
  Matrix<Scalar, Dynamic, 1> joints(toUType(Joints::count));
  for (size_t i = 0; i < toUType(Joints::count); ++i) {
    joints[i] = NAN;
  }
  joints[toUType(Joints::lAnkleRoll)] = aY;
  joints[toUType(Joints::rAnkleRoll)] = aY;
  return joints;
  //Scalar aX = atan2(diff[0], diff[2]);
  //Matrix<Scalar, 4, 4> baseTokickFoot =
  //  MathsUtils::getTInverse(getForwardEffector(baseLimb, eeIndex, type)) * getForwardEffector(baseLimb, eeIndex, type);
  //
  //Scalar lDiff =
}

/**
 * @Usage:
 * Matrix<Scalar, 3, 1> desComVel = Matrix<Scalar, 3, 1>::Zero();
 * Matrix<Scalar, 3, 1> desTorsoAngularVel = Matrix<Scalar, 3, 1>::Zero();
 * vector<unsigned> limbMotionSpace(toUType(LinkChains::count));
 * limbMotionSpace[toUType(LinkChains::head)] = 0; // Joint space
 * limbMotionSpace[toUType(LinkChains::lArm)] = 0; // Joint space
 * limbMotionSpace[toUType(LinkChains::rArm)] = 0; // Joint space
 * limbMotionSpace[CHAIN_L_LEG] = 1; // Cartesian space
 * limbMotionSpace[CHAIN_R_LEG] = 1; // Cartesian space
 * // Define desired velocities for each joint of limbs other than support limb
 * vector<Matrix<Scalar, Dynamic, 1>> limbVelocitiesD(toUType(LinkChains::count));
 * // Zero velocity in joint space
 * limbVelocitiesD[toUType(LinkChains::head)] = Matrix<Scalar, 2, 1>::Zero();
 * // Zero velocity in joint space
 * limbVelocitiesD[toUType(LinkChains::lArm)] = Matrix<Scalar, 5, 1>::Zero();
 * // Zero velocity in joint space
 * limbVelocitiesD[toUType(LinkChains::rArm)] = Matrix<Scalar, 5, 1>::Zero();
 * // Zero velocity in cartesian space (meaning for end-effector pose)
 * limbVelocitiesD[CHAIN_L_LEG] = Matrix<Scalar, 6, 1>::Zero();
 * limbVelocitiesD[CHAIN_R_LEG] = Matrix<Scalar, 6, 1>::Zero();
 * vector<int> eeIndices(toUType(LinkChains::count));
 * eeIndices[toUType(LinkChains::head)] = 0; // default
 * eeIndices[toUType(LinkChains::lArm)] = 0; // default
 * eeIndices[toUType(LinkChains::rArm)] = 0; // default
 * eeIndices[CHAIN_L_LEG] = toUType(LegEEs::footBase);
 * eeIndices[CHAIN_R_LEG] = toUType(LegEEs::footBase);
 * Matrix<Scalar, Dynamic, 1> jointsD =
 *   solveComIK(
 *     CHAIN_L_LEG,
 *     comVelocityD,
 *     limbMotionSpace,
 *     limbVelocitiesD,
 *     eeIndices,
 *     JointStateType::actual
 *   );
 */
template <typename Scalar>
Matrix<Scalar, Dynamic, 1>
KinematicsModule<Scalar>::solveComIK(
  const LinkChains& baseLimb,
  Matrix<Scalar, 6, 1>& comVelocityD,
  const vector<unsigned>& limbMotionSpace,
  const vector<Matrix<Scalar, Dynamic, 1> >& limbVelocitiesD,
  const vector<int>& eeIndices,
  const JointStateType& type)
{
  ASSERT(limbMotionSpace.size() == toUType(LinkChains::count));
  ASSERT(eeIndices.size() == toUType(LinkChains::count));
  ASSERT(baseLimb == LinkChains::lLeg || baseLimb == LinkChains::rLeg);

  static Matrix<Scalar, 3, 3> identity33 = Matrix<Scalar, 3, 3>::Identity();
  static Matrix<Scalar, 3, 3> zero33 = Matrix<Scalar, 3, 3>::Zero();
  Matrix<Scalar, 3, 3> globalToBodyRot =
    getGlobalToBody().block(0, 0, 3, 3);
  Matrix<Scalar, 6, 6> xBase;
  xBase <<
    identity33,
    MathsUtils::makeSkewMat(static_cast<Matrix<Scalar, 3, 1>>(globalToBodyRot * getBodyToGlobal().block(0, 3, 3, 1))),
    zero33,
    identity33;
  Matrix<Scalar, 6, Dynamic> baseJ =
    computeLimbJ(static_cast<LinkChains>(baseLimb), eeIndices[toUType(baseLimb)]);
  Matrix<Scalar, 3, 1> com =
    getComStateWrtFrame(static_cast<LinkChains>(baseLimb), eeIndices[toUType(baseLimb)]).position;
  Matrix<Scalar, 3, Dynamic> baseComJ =
    computeLimbComJ(static_cast<LinkChains>(baseLimb), type);
  Matrix<Scalar, 3, Dynamic> fsemJ =
    - baseJ.topRows(3) + MathsUtils::makeSkewMat(com) * baseJ.bottomRows(3) + baseComJ;


  Matrix<Scalar, 6, Dynamic> baseJT = xBase * baseJ;
  vector<Matrix<Scalar, 6, 6> > XiWithBaseJacobian(toUType(LinkChains::count));
  vector<Matrix<Scalar, Dynamic, Dynamic> > limbJsInv(toUType(LinkChains::count));
  Matrix<Scalar, 3, 1> comLimbsDiff = Matrix<Scalar, 3, 1>::Zero();
  for (const auto& lc : LinkChains()) {
    if (lc == baseLimb)
      continue;
    Matrix<Scalar, 3, Dynamic> limbComJ = globalToBodyRot * computeLimbComJ(lc, type);
    if (limbMotionSpace[toUType(lc)]) {
      Matrix<Scalar, 4, 4> limbT =
        getForwardEffector(lc, eeIndices[toUType(lc)], type);
      Matrix<Scalar, 6, 6> Xi;
      Xi <<
        identity33,
        MathsUtils::makeSkewMat(static_cast<Matrix<Scalar, 3, 1>>(globalToBodyRot * limbT.block(0, 3, 3, 1))),
        zero33,
        identity33;
      XiWithBaseJacobian[toUType(lc)] = Xi.inverse() * baseJT;
      Matrix<Scalar, Dynamic, Dynamic> limbJ = computeLimbJ(lc, eeIndices[toUType(lc)], type);
      limbJsInv[toUType(lc)] = MathsUtils::pseudoInverse(limbJ);
      fsemJ += limbComJ * limbJsInv[toUType(lc)] * XiWithBaseJacobian[toUType(lc)];
      comLimbsDiff += limbComJ * limbJsInv[toUType(lc)] * limbVelocitiesD[toUType(lc)];
    } else {
      comLimbsDiff += limbComJ * limbVelocitiesD[toUType(lc)];
    }
  }

  comVelocityD.segment(0, 3) = comVelocityD.segment(0, 3) - comLimbsDiff;
  Matrix<Scalar, Dynamic, 1> jointD(toUType(Joints::count));
  Matrix<Scalar, Dynamic, Dynamic> totalJ;
  totalJ.resize(6, 6);
  totalJ << fsemJ, -baseJ.bottomRows(3);
  vector<Matrix<Scalar, Dynamic, 1> > jointVD(toUType(LinkChains::count));
  jointVD[toUType(baseLimb)] =
    MathsUtils::pseudoInverse(totalJ) * comVelocityD;
  for (size_t i = 0; i < toUType(LinkChains::count); ++i) {
    if (i != toUType(baseLimb)) {
      jointVD[i].resize(linkChains[i]->size);
      jointVD[i].setZero();
      if (limbMotionSpace[i]) {
        Matrix<Scalar, Dynamic, Dynamic> rhs = limbVelocitiesD[i] + XiWithBaseJacobian[i] * jointVD[toUType(baseLimb)];
        jointVD[i] = limbJsInv[i] * rhs;
      }
    }
    for (size_t j = linkChains[i]->start; j < linkChains[i]->start + linkChains[i]->size; ++j) {
      jointD[j] = JOINT_STATE(j, type)->position() + jointVD[i][j-linkChains[i]->start] * cycleTime;
    }
  }
  return jointD;


  /*vector<Matrix<Scalar, 4, 4> > limbTs(toUType(LinkChains::count));
  Matrix<Scalar, 3, 3> rBase;
  Matrix<Scalar, 6, 6> XBase;
  Matrix<Scalar, 6, 6> Xo;
  vector<Matrix<Scalar, 3, Dynamic> > limbComJs(toUType(LinkChains::count));
  vector<Matrix<Scalar, Dynamic, Dynamic> > limbJs(toUType(LinkChains::count));
  vector<Matrix<Scalar, 6, 6> > XiWithBaseJacobian(toUType(LinkChains::count));
  vector<Matrix<Scalar, Dynamic, Dynamic> > limbJsInv(toUType(LinkChains::count));
  Matrix<Scalar, 3, 1> comWrtBaseLimb =
    getComStateWrtFrame(baseLimb, eeIndices[toUType(baseLimb)]).position;
  limbTs[toUType(baseLimb)] = getForwardEffector(baseLimb, eeIndices[toUType(baseLimb)], type);
  rBase = MathsUtils::getTInverse(limbTs[toUType(baseLimb)]).block(0, 0, 3, 3);
  Matrix<Scalar, 3, 1> eeBase = limbTs[toUType(baseLimb)].block(0, 3, 3, 1);
  Matrix<Scalar, 3, 1> eeBaseR = rBase * eeBase;
  XBase << Matrix<Scalar, 3, 3>::Identity(), MathsUtils::makeSkewMat(eeBaseR),
           Matrix<Scalar, 3, 3>::Zero(), Matrix<Scalar, 3, 3>::Identity();
  Xo << rBase, Matrix<Scalar, 3, 3>::Zero(), Matrix<Scalar, 3, 3>::Zero(), rBase;
  Matrix<Scalar, Dynamic, Dynamic> baseComJ;
  baseComJ.resize(6, linkChains[toUType(baseLimb)]->size);
  baseComJ.setZero();
  Matrix<Scalar, 6, 1> baseComVelocityD = comVelocityD;
  limbJs[toUType(baseLimb)] = Xo * computeLimbJ(baseLimb, eeIndices[toUType(baseLimb)], type);
  Matrix<Scalar, Dynamic, Dynamic> baseJT = XBase * limbJs[toUType(baseLimb)];
  Matrix<Scalar, 3, 1> comLimbsDiff = Matrix<Scalar, 3, 1>::Zero();
  for (size_t i = 0; i < toUType(LinkChains::count); ++i) {
    //cout << "limbMotionSpace[i];" << limbMotionSpace[i] << endl;
    limbComJs[i] = rBase * computeLimbComJ(static_cast<LinkChains>(i), type);
    if (limbMotionSpace[i]) {
      if (i == toUType(baseLimb)) {
        baseComJ.block(3, 0, 3, linkChains[i]->size) =
          -limbJs[i].block(3, 0, 3, linkChains[i]->size);
        baseComJ.block(0, 0, 3, linkChains[i]->size) =
          baseComJ.block(0, 0, 3, linkChains[i]->size) +
          -limbJs[i].block(0, 0, 3,linkChains[i]->size) +
          MathsUtils::makeSkewMat(comWrtBaseLimb) *
          limbJs[i].block(3, 0, 3,linkChains[i]->size) +
          limbComJs[i];
        //cout << "limbComJs:\n" << limbComJs[i] << endl;
        //cout << "comSkewMat:\n" << MathsUtils::makeSkewMat(comWrtBaseLimb) << endl;
        //cout << "-jv:\n" << -limbJs[i].block(0, 0, 3,linkChains[i]->size) << endl;
        //cout << "jw:\n" << limbJs[i].block(3, 0, 3,linkChains[i]->size)  << endl;
        //cout << "baseComJ:\n" << baseComJ.block(0, 0, 3, linkChains[i]->size) << endl;
      } else {
        limbTs[i] = getForwardEffector(static_cast<LinkChains>(i), eeIndices[i], type);
        limbJs[i] = Xo * computeLimbJ(static_cast<LinkChains>(i), eeIndices[i], type);
        Matrix<Scalar, 3, 1> ee = limbTs[i].block(0, 3, 3, 1);
        Matrix<Scalar, 6, 6> Xi;
        Matrix<Scalar, 3, 1> eeR = rBase * ee;
        Xi << Matrix<Scalar, 3, 3>::Identity(), MathsUtils::makeSkewMat(eeR), Matrix<Scalar, 3, 3>::Zero(), Matrix<Scalar, 3, 3>::Identity();
        XiWithBaseJacobian[i] = Xi.inverse() * baseJT;
        limbJsInv[i] = MathsUtils::pseudoInverse(limbJs[i]);
        baseComJ.block(0, 0, 3, linkChains[toUType(baseLimb)]->size) =
          baseComJ.block(0, 0, 3, linkChains[toUType(baseLimb)]->size) +
          limbComJs[i] * limbJsInv[i] * XiWithBaseJacobian[i];
        comLimbsDiff =
          comLimbsDiff + limbComJs[i] * limbJsInv[i] * limbVelocitiesD[i];
      }
    } else {
      comLimbsDiff =
        comLimbsDiff + limbComJs[i] * limbVelocitiesD[i];
    }
  }
  baseComVelocityD.segment(0, 3) =
    baseComVelocityD.segment(0, 3) - comLimbsDiff;
  Matrix<Scalar, Dynamic, 1> jointD(toUType(Joints::count));
  vector<Matrix<Scalar, Dynamic, 1> > jointVD(toUType(LinkChains::count));
  //cout << "baseComJ: " << baseComJ << endl;
  //cout << "baseComVelocityD: " << baseComVelocityD << endl;
  jointVD[toUType(baseLimb)] = MathsUtils::pseudoInverse(baseComJ, (Scalar)1e-2) * baseComVelocityD;
  for (size_t i = 0; i < toUType(LinkChains::count); ++i) {
    if (i != toUType(baseLimb)) {
      jointVD[i].resize(linkChains[i]->size);
      jointVD[i].setZero();
      if (limbMotionSpace[i]) {
        Matrix<Scalar, Dynamic, Dynamic> rhs = limbVelocitiesD[i] + XiWithBaseJacobian[i] * jointVD[toUType(baseLimb)];
        jointVD[i] = limbJsInv[i] * rhs;
      }
    }
    //cout << "jointVd:" << i << "\n" << jointVD[i] << endl;
    for (size_t j = linkChains[i]->start; j < linkChains[i]->start + linkChains[i]->size; ++j) {
      jointD[j] = JOINT_STATE(j, type)->position() + jointVD[i][j-linkChains[i]->start] * cycleTime;
      //if (i == toUType(baseLimb)) {
      //  cout << "j: " << j << endl;
      //  cout << "JOINT_STATE(j, type)->position: " << JOINT_STATE(j, type)->position << endl;
      //  cout << "jointD[j]: " << jointD[j] << endl;
      //}
    }
  }
  //cout << "jointsVD: " << jointVD[toUType(baseLimb)] << endl;
  return jointD;*/
}

template <typename Scalar>
Matrix<Scalar, Dynamic, Dynamic>
KinematicsModule<Scalar>::computeMassMatrix(
  const LinkChains& chainIndex, const JointStateType& type)
{
  size_t size = linkChains[toUType(chainIndex)]->size;
  size_t chainStart = linkChains[toUType(chainIndex)]->start;
  Matrix<Scalar, 3, Dynamic> jacobianCV;
  Matrix<Scalar, 3, Dynamic> jacobianCW;
  Matrix<Scalar, Dynamic, Dynamic> massMatrix;
  massMatrix.resize(size, size);
  massMatrix.setZero();
  for (size_t j = 0; j < size; ++j) {
    computeLinkComJ(static_cast<Links>(chainStart + j), jacobianCV, jacobianCW, type);
    massMatrix +=
      links[chainStart + j]->mass * jacobianCV.transpose() *jacobianCV +
      jacobianCW.transpose() * links[chainStart + j]->inertia * jacobianCW;
  }
  return massMatrix;
}

template <typename Scalar>
bool
KinematicsModule<Scalar>::computeVirtualMass(
  const LinkChains& chainIndex, const Matrix<Scalar, 3, 1>& direction,
  const Matrix<Scalar, 4, 4>& endEffector, Scalar& virtualMass, const JointStateType& type)
{
  //Matrix<Scalar, Dynamic, 1> joints = getJointPositions(chainStart, size, type);
  /////< center of mass of final link
  //Matrix<Scalar, Dynamic, Dynamic> T = MathsUtils::getTInverse(linkChains[chainIndex]->endT);
  //Matrix<Scalar, 4, 1> lastCom = T * linkComs[chainStart + size - 1];
  //cout << "lastCom: " << lastCom << endl;

  ///< Inertia matrix (size x size) at given joint configuration
  Matrix<Scalar, Dynamic, Dynamic> massMatrix = computeMassMatrix(chainIndex, type);
  //cout << "Mass matrix: " << endl << massMatrix << endl;

  //Matrix<Scalar, Dynamic, Dynamic> jacobian = computeLimbJ(chainIndex, lastCom, type);
  ///< Jacobian matrix (6 x size) at given joint configuration

  Matrix<Scalar, Dynamic, Dynamic> jacobianEE = computeLimbJ(chainIndex, endEffector, type, false);

  ///< Inertia matrix inverse (size x size) at given joint configuration
  Matrix<Scalar, Dynamic, Dynamic> mmInv = massMatrix.inverse();

  ///< Inertial projection in cartesian space (6x6)
  ///< (G = 6x6 Symmetric) [G11, G12;G21, G22]
  //Matrix<Scalar, Dynamic, Dynamic> gMatrix = jacobian * mmInv * jacobian.transpose();
  //cout << " gMatrix: " << endl <<  gMatrix << endl;
  Matrix<Scalar, Dynamic, Dynamic> gMatrixEE = jacobianEE * mmInv * jacobianEE.transpose();

  ///< Position vector from center of mass to end effector
  //Matrix<Scalar, 3, 1> pos =
  //  endEffector.block(0, 3, 3, 1) - lastCom.block(0, 0, 3, 1);
  //Matrix<Scalar, 3, 3> skewPosT = MathsUtils::makeSkewMat(pos);
  //Matrix<Scalar, 3, 3> skewPos = skewPosT.transpose();
  ///< Conversion of cartesian space inertia matrix from center of mass
  ///< to the contact point using skewPos
  //Matrix<Scalar, 3, 3> g11 = gMatrix.block(0, 0, 3, 3);
  //Matrix<Scalar, 3, 3> g12 = gMatrix.block(0, 3, 3, 3);
  //Matrix<Scalar, 3, 3> g22 = gMatrix.block(3, 3, 3, 3);
  //Matrix<Scalar, Dynamic, Dynamic> transfMassMatrix =
  // g11 + skewPos * g12.transpose() + g12* skewPosT + skewPos * g22 * skewPosT;
  //cout << "G-matrix: " << endl << transfMassMatrix << endl;
  //cout << "G12: " << endl << g12 + skewPos *g22 << endl;
  //cout << "G22: " << endl << g22 << endl;
  Matrix<Scalar, Dynamic, Dynamic> g11 = gMatrixEE.block(0, 0, 3, 3);
  ///< Virtual Mass in the target direction
  //virtualMass = direction.transpose() * transfMassMatrix * direction;
  virtualMass = direction.transpose() * g11 * direction;
  if (virtualMass != 0) {
    virtualMass = 1.f / virtualMass;
    return true;
  } else return false;
}

template <typename Scalar>
Matrix<Scalar, Dynamic, 1>
KinematicsModule<Scalar>::newtonEulerForces(
  const LinkChains& chainIndex, const Matrix<Scalar, 3, 1>& extForces,
  const Matrix<Scalar, 3, 1>& extMoments, Matrix<Scalar, 3, 1>& totalForces, Matrix<Scalar, 3, 1>& totalMoments,
  const LinkChains& supportLeg, const JointStateType& type)
{
  //cout << "Solving newton euler equation : " << endl;
  unsigned chainSize = linkChains[toUType(chainIndex)]->size;
  unsigned chainStart = linkChains[toUType(chainIndex)]->start;
  ///<Forward Recursion
  Matrix<Scalar, 3, 1> zAxis(0, 0, 1);
  Matrix<Scalar, 3, 1> linAcc(0, 0, -Constants::gravity);
  Matrix<Scalar, 3, 1> angVel(0, 0, 0);
  Matrix<Scalar, 3, 1> angAcc(0, 0, 0);
  Matrix<Scalar, 3, 1> linAccCom(0, 0, 0);
  vector<Matrix<Scalar, 3, 1> > comForces(chainSize);
  vector<Matrix<Scalar, 3, 1> > comMoments(chainSize);

  ///< Rotating torso forces and moments to inertial frame situated at
  ///< the base support leg
  Matrix<Scalar, 4, 4> supportT = getForwardEffector(supportLeg, toUType(LegEEs::footBase), type);
  linAcc = MathsUtils::transformVector(supportT, linAcc);

  for (size_t i = 0; i < chainSize; ++i) {
    //cout << "i; " << i << endl;
    //cout << "pos " << JOINT_STATE(chainStart + i, type)->position << endl;
    //cout << "vel; " << JOINT_STATE(chainStart + i, type)->velocity() << endl;
    //cout << "acc; " << JOINT_STATE(chainStart + i, type)->accel << endl;
    Matrix<Scalar, 4, 4> tMat =  JOINT_T(chainStart + i, type);
    Matrix<Scalar, 3, 3> rotMat = tMat.block(0, 0, 3, 3).transpose(); // transposed
    Matrix<Scalar, 3, 1> transMat = tMat.block(0, 3, 3, 1);
    angVel = rotMat * angVel + zAxis * JOINT_STATE(chainStart + i, type)->velocity(); // good
    angAcc = rotMat * angAcc + (rotMat * angVel).cross(
      zAxis *  JOINT_STATE(chainStart + i, type)->velocity()) +
      zAxis * JOINT_STATE(chainStart + i, type)->accel();  // good
    linAcc = rotMat * (angAcc.cross(transMat) + angVel.cross(
      angVel.cross(transMat)) + linAcc);  // good
    Matrix<Scalar, 3, 1> comP = links[chainStart + i]->com.segment(0, 3);
    linAccCom =
      angAcc.cross(comP) + angVel.cross(angVel.cross(comP)) + linAcc;
    comForces[i] = links[chainStart + i]->mass * linAccCom;
    comMoments[i] = links[chainStart + i]->inertia * angVel + angVel.cross(
      links[chainStart + i]->inertia * angVel);
    /*cout << "pos: " << JOINT_STATE(chainStart + i, type)->position << endl;
    cout << "velocity: " << JOINT_STATE(chainStart + i, type)->velocity() << endl;
    cout << "accel: " << JOINT_STATE(chainStart + i, type)->accel() << endl;
    cout << "comForces[" << i << "]" << "      "<< comForces[i](0,0) << endl;
    cout << "comForces[" << i << "]" << "      "<< comForces[i](1,0) << endl;
    cout << "comForces[" << i << "]" << "      "<< comForces[i](2,0) << endl;
    cout << "comMoments[" << i << "]" << "      "<< comMoments[i](0,0) << endl;
    cout << "comMoments[" << i << "]" << "      "<< comMoments[i](1,0) << endl;
    cout << "comMoments[" << i << "]" << "      "<< comMoments[i](2,0) << endl;
    cout<< endl;*/
  }

  ///<Backward Recursion
  Matrix<Scalar, Dynamic, 1> jointTorques;
  jointTorques.resize(chainSize);
  Matrix<Scalar, 3, 1> f(0, 0, 0);
  Matrix<Scalar, 3, 1> n(0, 0, 0);
  for (size_t i = chainSize; i > 0; --i) {
    Matrix<Scalar, 4, 4> tMat;
    if (i == chainSize) {
      tMat.setIdentity();
      f = extForces;
      n = extMoments;
    } else {
      tMat =  JOINT_T(chainStart + i, type);
    }
    Matrix<Scalar, 3, 3> rotMat = tMat.block(0, 0, 3, 3);
    Matrix<Scalar, 3, 1> transMat = tMat.block(0, 3, 3, 1);
    Matrix<Scalar, 3, 1> comP = links[chainStart + i - 1]->com.segment(0, 3);
    n = comMoments[i-1] + rotMat * n + comP.cross(comForces[i-1]) + transMat.cross(rotMat * f);
    f = comForces[i-1] + rotMat * f;
    jointTorques[i-1] = n.transpose() * zAxis;
  }
  Matrix<Scalar, 3, 3> initRot = JOINT_T(chainStart, type).block(0, 0, 3, 3);
  f = initRot * f;
  n = initRot * n;
  //jointTorques[0] = n.transpose() * zAxis;
  /*cout << "forces" << f(0,0) << endl;
  cout << "forces" << f(1,0) << endl;
  cout << "forces" << f(2,0) << endl;
  cout << "moments" << n(0,0) << endl;
  cout << "moments" << n(1,0) << endl;
  cout << "moments" << n(2,0) << endl;*/

  ///< Relocating moments to torso origin frame
  n = n + Matrix<Scalar, 3, 1>(linkChains[toUType(chainIndex)]->startT.block(0, 3, 3, 1)).cross(f);
  // f remains the same

  totalMoments = n;
  totalForces = f;
  return jointTorques;
}

#ifdef ALLOW_SYMBOLIC_COMPUTATIONS
template <typename Scalar>
SymEngine::DenseMatrix KinematicsModule<Scalar>::newtonEulerForcesSym(
  const LinkChains& chainIndex,
  const SymEngine::DenseMatrix& extForces,
  const SymEngine::DenseMatrix& extMoments,
  SymEngine::DenseMatrix& totalForces,
  SymEngine::DenseMatrix& totalMoments,
  const unsigned& supportLeg)
{
  using SymEngine::print_stack_on_segfault;
  using SymEngine::RCP;
  using SymEngine::integer;
  using SymEngine::number;
  using SymEngine::DenseMatrix;
  using SymEngine::Basic;
  using SymEngine::symbol;
  using SymEngine::Symbol;

  unsigned chainSize = linkChains[toUType(chainIndex)]->size;
  unsigned chainStart = linkChains[toUType(chainIndex)]->start;
  ///<Forward Recursion
  DenseMatrix zAxis = DenseMatrix(3, 1, {number(0), number(0), number(1)});
  DenseMatrix linAcc = DenseMatrix(3, 1, {number(0), number(0), number(-Constants::gravity)});
  DenseMatrix angVel = DenseMatrix(3, 1, {number(0), number(0), number(0)});
  DenseMatrix angAcc = DenseMatrix(3, 1, {number(0), number(0), number(0)});
  DenseMatrix linAccCom = DenseMatrix(3, 1, {number(0), number(0), number(0)});
  vector<DenseMatrix> comForces(chainSize);
  vector<DenseMatrix> comMoments(chainSize);
  ///< Rotating torso forces and moments to inertial frame situated at
  ///< the base support leg
  DenseMatrix supportT = eigenToSym(getForwardEffector(supportLeg, toUType(LegEEs::footBase)));
  linAcc = transformVector(supportT, linAcc);

  for (size_t i = 0; i < chainSize; ++i) {
    comForces[i] = DenseMatrix(3, 1);
    comMoments[i] = DenseMatrix(3, 1);
    DenseMatrix tMat = joints[chainStart+i]->symTrans;
    DenseMatrix rotMat = DenseMatrix(3, 3);
    tMat.submatrix(rotMat, 0, 0, 2, 2);
    DenseMatrix transMat = DenseMatrix(3, 1);
    tMat.submatrix(transMat, 0, 3, 2, 3);
    rotMat.mul_matrix(angVel, angVel);
    DenseMatrix temp0 = DenseMatrix(3, 1);
    DenseMatrix temp01 = DenseMatrix(3, 1);
    zAxis.mul_scalar(joints[chainStart+i]->symVel, temp0);
    angVel.add_matrix(temp0, angVel); // good
    rotMat.mul_matrix(angAcc, angAcc);
    DenseMatrix temp1 = DenseMatrix(3, 1);
    DenseMatrix temp2 = DenseMatrix(3, 1);
    rotMat.mul_matrix(angVel, temp1);
    SymEngine::cross(temp1, temp0, temp2);
    zAxis.mul_scalar(joints[chainStart+i]->symAcc, temp01);
    temp2.add_matrix(temp01, temp2);
    angAcc.add_matrix(temp2, angAcc);

    //cout << "angAcc\n" << angAcc << endl;

    DenseMatrix temp3 = DenseMatrix(3, 1);
    DenseMatrix temp4 = DenseMatrix(3, 1);
    DenseMatrix temp5 = DenseMatrix(3, 1);
    SymEngine::cross(angAcc, transMat, temp3);
    SymEngine::cross(angVel, transMat, temp4);
    SymEngine::cross(angVel, temp4, temp5);
    temp3.add_matrix(temp5, temp3);
    rotMat.mul_matrix(temp3, temp3);
    linAcc.add_matrix(temp3, linAcc);

    DenseMatrix comP = eigenToSym(links[chainStart + i]->com.segment(0, 3));

    DenseMatrix temp6 = DenseMatrix(3, 1);
    DenseMatrix temp7 = DenseMatrix(3, 1);
    DenseMatrix temp8 = DenseMatrix(3, 1);
    SymEngine::cross(angAcc, comP, temp6);
    SymEngine::cross(angVel, comP, temp7);
    SymEngine::cross(angVel, temp7, temp8);
    temp6.add_matrix(temp8, temp6);
    linAcc.add_matrix(temp6, linAccCom);
    //cout << "linAccCom\n" << linAccCom << endl;

    linAccCom.add_scalar(number(links[chainStart + i]->mass), comForces[i]);
    //cout << "comForces[i]\n" << comForces[i] << endl;
    DenseMatrix temp9 = DenseMatrix(3, 1);
    DenseMatrix temp10 = DenseMatrix(3, 1);
    eigenToSym(links[chainStart + i]->inertia).mul_matrix(angVel, temp9);
    SymEngine::cross(angVel, temp9, temp10);
    temp9.add_matrix(temp10, comMoments[i]);
    //cout << "comMoments[i]\n" << comMoments[i] << endl;
  }

  ///<Backward Recursion
  DenseMatrix jointTorques = DenseMatrix(chainSize, 1);
  DenseMatrix f = DenseMatrix(3, 1, {number(0), number(0), number(0)});
  DenseMatrix n = DenseMatrix(3, 1, {number(0), number(0), number(0)});
  for (size_t i = chainSize; i > 0; --i) {
    DenseMatrix tMat = DenseMatrix(4, 4);
    if (i == chainSize) {
      setIdentitySym(tMat);
      f = extForces;
      n = extMoments;
    } else {
      tMat = joints[chainStart+i]->symTrans;
    }
    DenseMatrix rotMat = DenseMatrix(3, 3);
    tMat.submatrix(rotMat, 0, 0, 2, 2);
    DenseMatrix transMat = DenseMatrix(3, 1);
    tMat.submatrix(transMat, 0, 3, 2, 3);
    DenseMatrix comP = eigenToSym(links[chainStart + i - 1]->com.segment(0, 3));
    DenseMatrix temp1 = DenseMatrix(3, 1);
    DenseMatrix temp2 = DenseMatrix(3, 1);
    DenseMatrix temp3 = DenseMatrix(3, 1);
    DenseMatrix temp4 = DenseMatrix(3, 1);
    rotMat.mul_matrix(n, temp1);
    SymEngine::cross(comP, comForces[i-1], temp2);
    rotMat.mul_matrix(f, temp3);
    SymEngine::cross(transMat, temp3, temp4);
    temp1.add_matrix(temp2, temp1);
    temp1.add_matrix(temp4, temp1);
    comMoments[i-1].add_matrix(temp1, n);
    comForces[i-1].add_matrix(temp3, f);
    DenseMatrix temp5 = DenseMatrix(1, 3);
    DenseMatrix temp6 = DenseMatrix(1, 1);
    n.transpose(temp5);
    temp5.mul_matrix(zAxis, temp6);
    jointTorques.set(i-1, 0, temp6.get(0, 0));
  }
  DenseMatrix initRot = eigenToSym(JOINT_T(chainStart, JointStateType::actual).block(0, 0, 3, 3));
  initRot.mul_matrix(f, f);
  initRot.mul_matrix(n, n);
  //jointTorques[0] = n.transpose() * zAxis;
  ///< Relocating moments to torso origin frame
  DenseMatrix initTrans = eigenToSym(Matrix<Scalar, 3, 1>(linkChains[toUType(chainIndex)]->startT.block(0, 3, 3, 1)));
  DenseMatrix temp1 = DenseMatrix(3, 1);
  SymEngine::cross(initTrans, f, temp1);
  n.add_matrix(temp1, n);
  // f remains the same
  totalMoments = n;
  totalForces = f;
  //cout << "totalForces\n:" << totalForces << endl;
  DenseMatrix temp;
  SymEngine::diff(totalForces, joints[chainStart]->symPos, temp);
  //cout << "totalForces\n:" << temp << endl;
  return jointTorques;
}
#endif

template <typename Scalar>
Matrix<Scalar, 4, 4> KinematicsModule<Scalar>::getGlobalToOther()
{
  auto other = globalBase == RobotFeet::lFoot ? RobotFeet::rFoot : RobotFeet::lFoot;
  return
    getGlobalToBody() * getForwardEffector(static_cast<LinkChains>(other), toUType(globalEnd));
}

template <typename Scalar>
Matrix<Scalar, 2, 1> KinematicsModule<Scalar>::computeZmp(
  const LinkChains& supportLeg,
  const JointStateType& type)
{
  Matrix<Scalar, Dynamic, 1> torque;
  return computeZmp(supportLeg, type, torque);
}

template <typename Scalar>
Matrix<Scalar, 2, 1> KinematicsModule<Scalar>::computeZmpWrtForces(
  const LinkChains& supportLeg,
  Matrix<Scalar, 3, 1>& torsoForces,
  Matrix<Scalar, 3, 1>& torsoMoments,
  const JointStateType& type)
{
  //cout << "torsoForces:" << torsoForces << endl;
  //cout << "torsoMoments:" << torsoMoments << endl;
  ///< Rotating torso forces and moments to inertial frame situated at
  ///< the base support leg
  Matrix<Scalar, 4, 4> supportT;
  supportT =
    MathsUtils::getTInverse(
      getForwardEffector(supportLeg, toUType(LegEEs::footBase), type)
    );

  Matrix<Scalar, 3, 3> supportR = supportT.block(0, 0, 3, 3);
  torsoForces = supportR * torsoForces;
  torsoMoments = supportR * torsoMoments;

  ///< Torso weight and vector
  Matrix<Scalar, 3, 1> torsoCog, batteryCog;
  torsoCog = (supportT * links[toUType(Links::torso)]->com).block(0, 0, 3, 1); // Torso center of mass
  //batteryCog = (supportT * Matrix<Scalar, 4, 1>(batteryX, batteryY, batteryZ, 1.f)).block(0, 0, 3, 1); // Battery center of mass
  Matrix<Scalar, 3, 1> torsoWeight(0.f, 0.f, links[toUType(Links::torso)]->mass * -Constants::gravity);
  //auto batteryWeight = Matrix<Scalar, 3, 1>(0.f, 0.f, batteryMass * -Constants::gravity);

  ///< Resulatant moments and forces at the base frame
  torsoMoments =
    torsoMoments +
    Matrix<Scalar, 3, 1>(supportT.block(0, 3, 3, 1)).cross(torsoForces) +
    torsoCog.cross(torsoWeight);//
    //batteryCog.cross(batteryWeight);

  Matrix<Scalar, 3, 1> rForce = -torsoForces - torsoWeight;// - batteryWeight;

  // zmp_y * R_z - zmp_z * R_y + M_x = 0
  // zmp_z * R_x - zmp_x * R_z + M_y = 0
  // zmp_x * R_y - zmp_y * R_x + M_z = 0

  Matrix<Scalar, 2, 1> zmp;
  zmp[0] = torsoMoments[1] / rForce[2]; // M_y / R_z
  zmp[1] = - torsoMoments[0] / rForce[2]; // - M_x / R_z
  return zmp;
}


template <typename Scalar>
Matrix<Scalar, 2, 1> KinematicsModule<Scalar>::computeZmpWrtChain(
  const LinkChains& supportLeg,
  const LinkChains& chainIndex,
  const JointStateType& type,
  Matrix<Scalar, 3, 1>& torsoForces,
  Matrix<Scalar, 3, 1>& torsoMoments,
  Matrix<Scalar, Dynamic, 1>& torques)
{
  Matrix<Scalar, 3, 1> extForces;
  Matrix<Scalar, 3, 1> extMoments;
  Matrix<Scalar, 3, 1> chainForces;
  Matrix<Scalar, 3, 1> chainMoments;
  extForces.setZero();
  extMoments.setZero();
  unsigned chainSize = linkChains[toUType(chainIndex)]->size;
  unsigned chainStart = linkChains[toUType(chainIndex)]->start;
  torques.resize(linkChains[toUType(chainIndex)]->size);
  Matrix<Scalar, Dynamic, 1> torque =
    newtonEulerForces(
      chainIndex, extForces, extMoments, chainForces, chainMoments, supportLeg, type);
  torsoForces += chainForces;
  torsoMoments += chainMoments;
  torques.block(chainStart, 0, chainSize, 1) = torque;
  ///< Rotating torso forces and moments to inertial frame situated at
  ///< the base support leg
  Matrix<Scalar, 4, 4> supportT;
  supportT =
    MathsUtils::getTInverse(
      getForwardEffector(supportLeg, toUType(LegEEs::footBase), type)
    );

  Matrix<Scalar, 3, 3> supportR = supportT.block(0, 0, 3, 3);
  torsoForces = supportR * torsoForces;
  torsoMoments = supportR * torsoMoments;

  ///< Torso weight and vector
  Matrix<Scalar, 3, 1> torsoCog, batteryCog;
  torsoCog = (supportT * links[toUType(Links::torso)]->com).block(0, 0, 3, 1); // Torso center of mass
  //batteryCog = (supportT * Matrix<Scalar, 4, 1>(batteryX, batteryY, batteryZ, 1.f)).block(0, 0, 3, 1); // Battery center of mass
  Matrix<Scalar, 3, 1> torsoWeight(0.f, 0.f, links[toUType(Links::torso)]->mass * -Constants::gravity);
  //auto batteryWeight = Matrix<Scalar, 3, 1>(0.f, 0.f, batteryMass * -Constants::gravity);

  ///< Resulatant moments and forces at the base frame
  torsoMoments =
    torsoMoments +
    Matrix<Scalar, 3, 1>(supportT.block(0, 3, 3, 1)).cross(torsoForces) +
    torsoCog.cross(torsoWeight);//
    //batteryCog.cross(batteryWeight);

  Matrix<Scalar, 3, 1> rForce = -torsoForces - torsoWeight;// - batteryWeight;

  // zmp_y * R_z - zmp_z * R_y + M_x = 0
  // zmp_z * R_x - zmp_x * R_z + M_y = 0
  // zmp_x * R_y - zmp_y * R_x + M_z = 0

  Matrix<Scalar, 2, 1> zmp;
  zmp[0] = torsoMoments[1] / rForce[2]; // M_y / R_z
  zmp[1] = - torsoMoments[0] / rForce[2]; // - M_x / R_z

  //Matrix<Scalar, 2, 1> com;
  //computeComWrtBase(supportLeg, toUType(LegEEs::footBase), com, type);
  //cout << "Center of mass: " << endl;
  //cout << com << endl;

  //cout << "Zmp: " << endl;
  //cout << zmp << endl;
  return zmp;
}

template <typename Scalar>
Matrix<Scalar, 2, 1> KinematicsModule<Scalar>::computeZmp(
  const LinkChains& supportLeg,
  const JointStateType& type,
  Matrix<Scalar, Dynamic, 1>& torques)
{
  Matrix<Scalar, 3, 1> extForces;
  Matrix<Scalar, 3, 1> extMoments;
  Matrix<Scalar, 3, 1> torsoForces;
  Matrix<Scalar, 3, 1> torsoMoments;
  extForces.setZero();
  extMoments.setZero();
  torsoForces.setZero();
  torsoMoments.setZero();
  torques.resize(toUType(Joints::count));
  for (int i = 0; i < toUType(LinkChains::count); ++i) {
    size_t chainSize = linkChains[i]->size;
    size_t chainStart = linkChains[i]->start;
    Matrix<Scalar, 3, 1> chainForces;
    Matrix<Scalar, 3, 1> chainMoments;
    Matrix<Scalar, Dynamic, 1> torque =
      newtonEulerForces(
        static_cast<LinkChains>(i), extForces, extMoments, chainForces, chainMoments, supportLeg, type);
    torsoForces += chainForces;
    torsoMoments += chainMoments;
    torques.block(chainStart, 0, chainSize, 1) = torque;
  }
  //cout << "torsoForces:" << torsoForces << endl;
  //cout << "torsoMoments:" << torsoMoments << endl;
  ///< Rotating torso forces and moments to inertial frame situated at
  ///< the base support leg
  Matrix<Scalar, 4, 4> supportT;
  supportT =
    MathsUtils::getTInverse(
      getForwardEffector(supportLeg, toUType(LegEEs::footBase), type)
    );

  Matrix<Scalar, 3, 3> supportR = supportT.block(0, 0, 3, 3);
  torsoForces = supportR * torsoForces;
  torsoMoments = supportR * torsoMoments;

  ///< Torso weight and vector
  Matrix<Scalar, 3, 1> torsoCog, batteryCog;
  torsoCog = (supportT * links[toUType(Links::torso)]->com).block(0, 0, 3, 1); // Torso center of mass
  //batteryCog = (supportT * Matrix<Scalar, 4, 1>(batteryX, batteryY, batteryZ, 1.f)).block(0, 0, 3, 1); // Battery center of mass
  Matrix<Scalar, 3, 1> torsoWeight(0.f, 0.f, links[toUType(Links::torso)]->mass * -Constants::gravity);
  //auto batteryWeight = Matrix<Scalar, 3, 1>(0.f, 0.f, batteryMass * -Constants::gravity);

  ///< Resulatant moments and forces at the base frame
  torsoMoments =
    torsoMoments +
    Matrix<Scalar, 3, 1>(supportT.block(0, 3, 3, 1)).cross(torsoForces) +
    torsoCog.cross(torsoWeight);//
    //batteryCog.cross(batteryWeight);

  Matrix<Scalar, 3, 1> rForce = -torsoForces - torsoWeight;// - batteryWeight;

  // zmp_y * R_z - zmp_z * R_y + M_x = 0
  // zmp_z * R_x - zmp_x * R_z + M_y = 0
  // zmp_x * R_y - zmp_y * R_x + M_z = 0

  Matrix<Scalar, 2, 1> zmp;
  zmp[0] = torsoMoments[1] / rForce[2]; // M_y / R_z
  zmp[1] = - torsoMoments[0] / rForce[2]; // - M_x / R_z

  Matrix<Scalar, 2, 1> com;
  computeComWrtBase(supportLeg, toUType(LegEEs::footBase), com, type);
  //cout << "Center of mass: " << endl;
  //cout << com << endl;

  //cout << "Zmp: " << endl;
  //cout << zmp << endl;
  return zmp;
}

template <typename Scalar>
Matrix<Scalar, 3, 1> KinematicsModule<Scalar>::calculateCenterOfMass(
  const JointStateType& type)
{
  Matrix<Scalar, 3, 1> com;
  com.setZero();
//  cout << "Calculating center of mass ..." << endl;
  for (size_t i = 0; i < toUType(Joints::count); ++i)
  {
    ///< Get center of mass position in base frame
    com += joints[i]->states[toUType(type)]->comInBase * joints[i]->link->mass;
    //cout << "JointCominBase: " << joints[i]->states[toUType(type)]->comInBase << endl;
    //cout << "JointMass: << joints[i]->link->mass: " << endl;
  }
  com += links[toUType(Links::torso)]->com.block(0, 0, 3, 1) * links[toUType(Links::torso)]->mass;
  //cout << "torso mass: " << links[toUType(Links::torso)]->mass << endl;
  //cout << "torso com: " << links[toUType(Links::torso)]->com.block(0, 0, 3, 1) << endl;
  //com = com + Matrix<Scalar, 3, 1>(batteryX, batteryY, batteryZ) * batteryMass;
  com = com / totalMassH25;
  //cout << "com: " << com << endl;
  return com;
}

template <typename Scalar>
void KinematicsModule<Scalar>::updateFootOnGround()
{
  #ifndef V6_CROSS_BUILD
    auto& fsrSensors = FSR_SENSORS_OUT(MotionModule);
  #else
    #ifndef REALTIME_LOLA_AVAILABLE
      auto& fsrSensors = FSR_SENSORS_OUT(MotionModule);
    #else
      const auto& fsrSensors = FSR_SENSORS_IN(MotionModule);
    #endif
  #endif

  /* Naoqi does it itself idk why?
   * fsrSensors[L_FOOT_TOTAL_WEIGHT] =
      fsrSensors[L_FOOT_FSR_FL] +
      fsrSensors[L_FOOT_FSR_FR] +
      fsrSensors[L_FOOT_FSR_RL] +
      fsrSensors[L_FOOT_FSR_RR];
  fsrSensors[R_FOOT_TOTAL_WEIGHT] =
      fsrSensors[R_FOOT_FSR_FL] +
      fsrSensors[R_FOOT_FSR_FR] +
      fsrSensors[R_FOOT_FSR_RL] +
      fsrSensors[R_FOOT_FSR_RR];
  fsrSensors[L_FOOT_COP_X] =
    LFSRFL_X * fsrSensors[L_FOOT_FSR_FL] +
    LFSRFR_X * fsrSensors[L_FOOT_FSR_FR] +
    LFSRRL_X * fsrSensors[L_FOOT_FSR_RL] +
    LFSRRR_X * fsrSensors[L_FOOT_FSR_RR];
  fsrSensors[L_FOOT_COP_X] /= fsrSensors[L_FOOT_TOTAL_WEIGHT];
  fsrSensors[L_FOOT_COP_Y] =
    LFSRFL_Y * fsrSensors[L_FOOT_FSR_FL] +
    LFSRFR_Y * fsrSensors[L_FOOT_FSR_FR] +
    LFSRRL_Y * fsrSensors[L_FOOT_FSR_RL] +
    LFSRRR_Y * fsrSensors[L_FOOT_FSR_RR];
  fsrSensors[L_FOOT_COP_Y] /= fsrSensors[L_FOOT_TOTAL_WEIGHT];
  fsrSensors[R_FOOT_COP_X] =
    RFSRFL_X * fsrSensors[R_FOOT_FSR_FL] +
    RFSRFR_X * fsrSensors[R_FOOT_FSR_FR] +
    RFSRRL_X * fsrSensors[R_FOOT_FSR_RL] +
    RFSRRR_X * fsrSensors[R_FOOT_FSR_RR];
  fsrSensors[R_FOOT_COP_X] /= fsrSensors[R_FOOT_TOTAL_WEIGHT];
  fsrSensors[R_FOOT_COP_Y] =
    RFSRFL_Y * fsrSensors[R_FOOT_FSR_FL] +
    RFSRFR_Y * fsrSensors[R_FOOT_FSR_FR] +
    RFSRRL_Y * fsrSensors[R_FOOT_FSR_RL] +
    RFSRRR_Y * fsrSensors[R_FOOT_FSR_RR];
  fsrSensors[R_FOOT_COP_Y] /= fsrSensors[R_FOOT_TOTAL_WEIGHT];*/
  Matrix<Scalar, 2, 1> feetForces;
  feetForces << fabsf(fsrSensors[toUType(FsrSensors::lFootTotalWeight)]),
                fabsf(fsrSensors[toUType(FsrSensors::rFootTotalWeight)]);
  feetForcesBuffer.push_back(feetForces);
  if (feetForcesBuffer.size() >= ffBufferSize) {
    Matrix<Scalar, 2, 1> bufferAvg = Matrix<Scalar, 2, 1>::Zero();
    for (int i = 0; i < feetForcesBuffer.size(); ++i) {
      bufferAvg = bufferAvg + feetForcesBuffer[i];
    }
    bufferAvg = bufferAvg / ffBufferSize;

    if (bufferAvg[toUType(RobotFeet::lFoot)] < 0.1 && bufferAvg[toUType(RobotFeet::rFoot)] < 0.1) {
      footOnGround = RobotFeet::unknown;
    } else {
      if (bufferAvg[toUType(RobotFeet::lFoot)] > 1.3) footOnGround = RobotFeet::lFoot;
      else if (bufferAvg[toUType(RobotFeet::lFoot)] > 1.3) footOnGround = RobotFeet::rFoot;
      else footOnGround = RobotFeet::lFoot;
    }
  }

  //! Override if we know the step leg
  if(STEP_LEG_OUT(MotionModule) == RobotFeet::lFoot)
    footOnGround = RobotFeet::rFoot;
  else if(STEP_LEG_OUT(MotionModule) == RobotFeet::rFoot)
    footOnGround = RobotFeet::lFoot;
  FOOT_ON_GROUND_OUT(MotionModule) = footOnGround;
}

template <typename Scalar>
Matrix<Scalar, 2, 1> KinematicsModule<Scalar>::computeFsrZmp(const RobotFeet& refFrame)
{
  #ifndef V6_CROSS_BUILD
    auto& fsrSensors = FSR_SENSORS_OUT(MotionModule);
  #else
    #ifndef REALTIME_LOLA_AVAILABLE
      auto& fsrSensors = FSR_SENSORS_OUT(MotionModule);
    #else
      const auto& fsrSensors = FSR_SENSORS_IN(MotionModule);
    #endif
  #endif
  Matrix<Scalar, 4, 4> tl = getForwardEffector(LinkChains::lLeg, toUType(LegEEs::ankle));
  Matrix<Scalar, 4, 4> tr = getForwardEffector(LinkChains::rLeg, toUType(LegEEs::ankle));
  Matrix<Scalar, 2, 1> cop;
  cop[0] =
    ((tl(0, 3) + fsrSensors[toUType(FsrSensors::lFootCopX)]) * fsrSensors[toUType(FsrSensors::lFootTotalWeight)] +
    (tr(0, 3) + fsrSensors[toUType(FsrSensors::rFootCopX)]) * fsrSensors[toUType(FsrSensors::rFootTotalWeight)]) /
    (fsrSensors[toUType(FsrSensors::lFootTotalWeight)] + fsrSensors[toUType(FsrSensors::rFootTotalWeight)]);
  cop[1] =
    ((tl(1, 3) + fsrSensors[toUType(FsrSensors::lFootCopY)]) * fsrSensors[toUType(FsrSensors::lFootTotalWeight)] +
    (tr(1, 3) + fsrSensors[toUType(FsrSensors::rFootCopY)]) * fsrSensors[toUType(FsrSensors::rFootTotalWeight)]) /
    (fsrSensors[toUType(FsrSensors::lFootTotalWeight)] + fsrSensors[toUType(FsrSensors::rFootTotalWeight)]);
  //LOG_INFO("cop: " << cop.transpose())
  if (refFrame == RobotFeet::lFoot) {
    cop[0] -= tl(0, 3);
    cop[1] -= tl(1, 3);
  } else if (refFrame == RobotFeet::rFoot) {
    cop[0] -= tr(0, 3);
    cop[1] -= tr(1, 3);
  }
  return cop;
}


template <typename Scalar>
Matrix<Scalar, 4, 4> KinematicsModule<Scalar>::getFeetCenterT()
{
  Matrix<Scalar, 4, 4> T;
  Matrix<Scalar, 4, 4> ee;
  if (footOnGround == RobotFeet::rFoot) {
    MathsUtils::makeTranslation(ee, Constants::footOriginShiftX, Constants::footSeparation / 2, 0.0);
    T = L_FOOT_TRANS_OUT(MotionModule) * ee;
  } else { ///< If left or unknown
    MathsUtils::makeTranslation(ee, Constants::footOriginShiftX, -Constants::footSeparation / 2, 0.0);
    T = R_FOOT_TRANS_OUT(MotionModule) * ee;
  }
  return T;
}

template <typename Scalar>
void KinematicsModule<Scalar>::updateTorsoToFeet()
{
  L_FOOT_TRANS_OUT(MotionModule) =
    getForwardEffector(LinkChains::lLeg, toUType(LegEEs::footCenter)).template cast <float> ();
  R_FOOT_TRANS_OUT(MotionModule) =
    getForwardEffector(LinkChains::rLeg, toUType(LegEEs::footCenter)).template cast <float> ();
  footSpacing =
    L_FOOT_TRANS_OUT(MotionModule)(1, 3) -
    R_FOOT_TRANS_OUT(MotionModule)(1, 3);
}

template <typename Scalar> Matrix<Scalar, 4, 1>
KinematicsModule<Scalar>::getWorldToCam(
  const CameraId& camIndex, const Matrix<Scalar, 4, 1>& posInFoot)
{
  if (camIndex == CameraId::headTop) {
    return UPPER_CAM_TRANS_OUT(MotionModule).cast <Scalar> () * posInFoot;
  } else if (camIndex == CameraId::headBottom) {
    return LOWER_CAM_TRANS_OUT(MotionModule).cast <Scalar> () * posInFoot;
  }
}

template <typename Scalar> void
KinematicsModule<Scalar>::updateFootToCamT()
{
  Matrix<Scalar, 4, 4> torsoToFeet = getFeetCenterT();
  Matrix<Scalar, 4, 4> torsoPitchRot;
  MathsUtils::makeRotationXYZ(
    torsoPitchRot,
    (Scalar) -torsoRollOffset * M_PI / 180,
    (Scalar) -torsoPitchOffset * M_PI / 180,
    (Scalar) 0.0);
  torsoToFeet = torsoPitchRot * torsoToFeet;
  UPPER_CAM_TRANS_OUT(MotionModule) =
    (MathsUtils::getTInverse(
      getForwardEffector(
        LinkChains::head, linkChains[toUType(LinkChains::head)]->endEffectors[toUType(CameraId::headTop)])
     ) * torsoToFeet
    ).template cast <float> ();
  LOWER_CAM_TRANS_OUT(MotionModule) =
    (MathsUtils::getTInverse(
      getForwardEffector(
        LinkChains::head, linkChains[toUType(LinkChains::head)]->endEffectors[toUType(CameraId::headBottom)])
     ) * torsoToFeet
    ).template cast <float> ();
}

template <typename Scalar> boost::shared_ptr<PostureTask<Scalar> >
KinematicsModule<Scalar>::makePostureTask(
  const Matrix<Scalar, Dynamic, 1>& targetJoints,
  vector<bool> activeJoints,
  const Scalar& weight,
  const Scalar& gain,
  vector<float> activeResidual)
{
  if (activeJoints.empty()) {
    activeJoints.resize(toUType(Joints::count), false);
    for (size_t i = 0; i < toUType(Joints::count); ++i)
      activeJoints[i] = true;
  }
  return boost::shared_ptr<PostureTask<Scalar> >(new PostureTask<Scalar>(
    targetJoints,
    weight,
    gain,
    activeJoints,
    motionModule->getKinematicsModule(),
    activeResidual
  ));
}

template <typename Scalar> boost::shared_ptr<ContactTask<Scalar> >
KinematicsModule<Scalar>::makeContactTask(
  const LinkChains& chainIndex,
  const unsigned& eeIndex,
  vector<bool> activeJoints,
  const Scalar& weight,
  const Scalar& gain,
  vector<float> activeResidual)
{
  if (activeJoints.empty()) {
    activeJoints.resize(toUType(Joints::count), false);
    for (size_t i = 0; i < linkChains[toUType(chainIndex)]->size; ++i)
      activeJoints[linkChains[toUType(chainIndex)]->start + i] = true;
  }
  return boost::shared_ptr<ContactTask<Scalar> >(new ContactTask<Scalar>(
    chainIndex,
    linkChains[toUType(chainIndex)]->endEffectors[eeIndex],
    weight,
    gain,
    activeJoints,
    motionModule->getKinematicsModule(),
    activeResidual
  ));
}

template <typename Scalar> boost::shared_ptr<CartesianTask<Scalar> >
KinematicsModule<Scalar>::makeCartesianTask(
  const LinkChains& chainIndex,
  const Matrix<Scalar, 4, 4>& ee,
  const Matrix<Scalar, 4, 4>& targetT,
  vector<bool> activeJoints,
  const Scalar& weight,
  const Scalar& gain,
  vector<float> activeResidual)
{
  if (activeJoints.empty()) {
    activeJoints.resize(toUType(Joints::count), false);
    for (size_t i = 0; i < linkChains[toUType(chainIndex)]->size; ++i)
      activeJoints[linkChains[toUType(chainIndex)]->start + i] = true;
  }
  return boost::shared_ptr<CartesianTask<Scalar> >(new CartesianTask<Scalar>(
    chainIndex,
    ee,
    targetT,
    weight,
    gain,
    activeJoints,
    motionModule->getKinematicsModule(),
    activeResidual
  ));
}

template <typename Scalar> boost::shared_ptr<CartesianTask<Scalar> >
KinematicsModule<Scalar>::makeCartesianTask(
  const LinkChains& chainIndex,
  const unsigned& eeIndex,
  const Matrix<Scalar, 4, 4>& targetT,
  vector<bool> activeJoints,
  const Scalar& weight,
  const Scalar& gain,
  vector<float> activeResidual)
{
  return makeCartesianTask(
    chainIndex,
    linkChains[toUType(chainIndex)]->endEffectors[eeIndex],
    targetT,
    activeJoints,
    weight,
    gain,
    activeResidual);
}

template <typename Scalar> boost::shared_ptr<ComTask<Scalar> >
KinematicsModule<Scalar>::makeComTask(
  // com reference frame (left or right foot defined by CHAIN_L_LEG or CHAIN_R_LEG)
  const RobotFeet& baseFrame,
  const LegEEs& baseEE,
  const Matrix<Scalar, 3, 1>& comTarget,
  vector<bool> activeJoints,
  const Scalar& weight,
  const Scalar& gain,
  vector<float> activeResidual)
{
  if (activeJoints.empty()) {
    activeJoints.resize(toUType(Joints::count), true);
  }
  return boost::shared_ptr<ComTask<Scalar> >(new ComTask<Scalar>(
    baseFrame,
    baseEE,
    comTarget,
    weight,
    gain,
    activeJoints,
    motionModule->getKinematicsModule(),
    activeResidual
  ));
}

template <typename Scalar> boost::shared_ptr<TorsoTask<Scalar> >
KinematicsModule<Scalar>::makeTorsoTask(
  // com reference frame (left or right foot defined by CHAIN_L_LEG or CHAIN_R_LEG)
  const RobotFeet& baseFrame,
  const LegEEs& baseEE,
  const Matrix<Scalar, 4, 4>& target,
  vector<bool> activeJoints,
  const Scalar& weight,
  const Scalar& gain,
  vector<float> activeResidual)
{
  if (activeJoints.empty()) {
    activeJoints.resize(toUType(Joints::count), true);
  }
  return boost::shared_ptr<TorsoTask<Scalar> >(new TorsoTask<Scalar>(
    baseFrame,
    baseEE,
    target,
    weight,
    gain,
    activeJoints,
    motionModule->getKinematicsModule(),
    activeResidual
  ));
}

template <typename Scalar> Matrix<Scalar, Dynamic, 1>
KinematicsModule<Scalar>::solveTasksIK(
  const vector<boost::shared_ptr<MotionTask<Scalar> > >& tasks,
  const unsigned& maxIterations)
{
  tis->reset(false);
  vector<bool> activeJoints(toUType(Joints::count), false);
  for (size_t i = 0; i < activeJoints.size(); ++i) {
    for (size_t j = 0; j < tasks.size(); ++j) {
      if (tasks[j]) {
        if (tasks[j]->getActiveJoints()[i])
          activeJoints[i] = true;
      }
    }
  }
  for (size_t i = 0; i < tasks.size(); ++i) {
    if (tasks[i])
      tis->addTask(tasks[i]);
  }
  tis->setActiveJoints(activeJoints);
  return tis->solve(maxIterations);
}

template <typename Scalar> Matrix<Scalar, Dynamic, 1>
KinematicsModule<Scalar>::solveCartesianIK(
  const LinkChains& chainIndex,
  const unsigned& eeIndex,
  const Matrix<Scalar, 4, 4>& targetT,
  const unsigned& maxIterations)
{
  vector<bool> activeJoints;
  if (activeJoints.empty()) {
    activeJoints.resize(toUType(Joints::count), false);
    for (size_t i = 0; i < linkChains[toUType(chainIndex)]->size; ++i)
      activeJoints[linkChains[toUType(chainIndex)]->start + i] = true;
  }
  TaskIkSolver<Scalar> tis =
    TaskIkSolver<Scalar>(
      motionModule->getKinematicsModule(), maxIterations, activeJoints, false, 1e-2);
  tis.init();
  CartesianTaskPtr ctp =
    boost::shared_ptr<CartesianTask<Scalar> >(new CartesianTask<Scalar>(
      chainIndex,
      linkChains[toUType(chainIndex)]->endEffectors[eeIndex],
      targetT,
      1,
      0.9,
      activeJoints,
      motionModule->getKinematicsModule()
    ));
  if (activeJoints.empty()) {
    activeJoints.resize(toUType(Joints::count), false);
    for (size_t i = 0; i < linkChains[toUType(chainIndex)]->size; ++i)
      activeJoints[linkChains[toUType(chainIndex)]->start + i] = true;
  }
  tis.addTask(ctp);
  return tis.solve(maxIterations);
}

template <typename Scalar>
Matrix<Scalar, Dynamic, 1>
KinematicsModule<Scalar>::solveJacobianIK(
  const LinkChains& chainIndex,
  const Matrix<Scalar, 4, 4>& endEffector,
  const Matrix<Scalar, 4, 4>& targetT,
  const unsigned& maxIterations,
  vector<bool>& activeJoints,
  vector<float> activeResidual)
{
  tis->reset(false);
  unsigned chainStart = linkChains[toUType(chainIndex)]->start;
  unsigned chainSize = linkChains[toUType(chainIndex)]->size;
  vector<bool> wbActiveJoints(toUType(Joints::count), false);
  for (size_t i = 0; i < chainSize; ++i) {
    wbActiveJoints[chainStart+i] = activeJoints[i];
  }
  auto ct = this->makeCartesianTask(chainIndex, endEffector, targetT, wbActiveJoints, 1.0, 0.95, activeResidual);
  tis->addTask(ct);
  tis->setActiveJoints(wbActiveJoints);
  tis->setMaxVelocityLimitGain(1.0);
  Matrix<Scalar, Dynamic, 1> res = tis->solve(maxIterations);
  Matrix<Scalar, Dynamic, 1> output;
  output.resize(chainSize);
  for (size_t i = 0; i < chainSize; ++i) {
    if (activeJoints[i])
      output[i] = res[chainStart + i];
    else
      output[i] = this->getJointPosition(static_cast<Joints>(chainStart + i));
  }
  return output;
}

template <typename Scalar> Matrix<Scalar, Dynamic, 1>
KinematicsModule<Scalar>::solveJacobianIK(
  const LinkChains& chainIndex, const unsigned& eeIndex, const Matrix<Scalar, 4, 4>& targetT,
  const unsigned& maxIterations, const JointStateType& startType, const bool& solveForOrientation,
  const Scalar& pTol,
  const Scalar& oTol,
  vector<bool>& activeJoints)
{
  return solveJacobianIK(
    chainIndex,
    linkChains[toUType(chainIndex)]->endEffectors[eeIndex],
    targetT,
    maxIterations,
    startType,
    solveForOrientation,
    pTol,
    oTol,
    activeJoints);
}

template <typename Scalar>
Matrix<Scalar, Dynamic, 1>
KinematicsModule<Scalar>::solveJacobianIK(
  const LinkChains& chainIndex,
  const Matrix<Scalar, 4, 4>& endEffector,
  const Matrix<Scalar, 4, 4>& targetT,
  const unsigned& maxIterations,
  const JointStateType& startType,
  const bool& solveForOrientation,
  const Scalar& pTol,
  const Scalar& oTol,
  vector<bool>& activeJoints)
{
  Matrix<Scalar, Dynamic, 1> startJoints =
    getJointPositions(Joints::first, toUType(Joints::count), startType);
  JointStateType type = JointStateType::sim;
  unsigned chainStart = linkChains[toUType(chainIndex)]->start;
  unsigned chainSize = linkChains[toUType(chainIndex)]->size;
  if (activeJoints.empty())
    activeJoints = vector<bool>(chainSize, true);
  Scalar kP = 0.85, kO = 1.0;
  if (startType != type)
    setStateFromTo(startType, type);
  Matrix<Scalar, 4, 4> initT =
    getForwardEffector(chainIndex, endEffector, type);
  Matrix<Scalar, 3, 1> initPos = initT.block(0, 3, 3, 1);
  Matrix<Scalar, 3, 1> targetPos = targetT.block(0, 3, 3, 1);
  Matrix<Scalar, 3, 1> diffPos = targetPos - initPos;
  Matrix<Scalar, 3, 1> diffOrient;
  if (solveForOrientation)
    diffOrient = MathsUtils::getOrientationDiff(initT, targetT);
  bool success = false;
  if (solveForOrientation) {
    if (diffPos.norm() < pTol && diffOrient.norm() < oTol) {
      success = true;
      return
        this->getJointPositions(
          static_cast<Joints>(chainStart), chainSize, startType);
    }
  } else {
    if (diffPos.norm() < pTol) {
      success = true;
      return
        this->getJointPositions(
          static_cast<Joints>(chainStart), chainSize, startType);
    }
  }
  for (size_t i = 0; i < maxIterations; ++i) {
    Matrix<Scalar, Dynamic, Dynamic> J, jInv;
    Matrix<Scalar, Dynamic, 1> result;
    if (solveForOrientation) {
      J = computeLimbJ(chainIndex, endEffector, type, false);
      for (size_t i = 0; i < chainSize; ++i) {
        if (!activeJoints[i]) {
          J.col(i).setZero();
        }
      }
      jInv = MathsUtils::pseudoInverse(J);
      Matrix<Scalar, 6, 1> diff;
      diff.block(0, 0, 3, 1) = kP * diffPos;
      diff.block(3, 0, 3, 1) = kO * diffOrient;
      result = jInv * diff;
    } else {
      J = computeLimbJ(chainIndex, endEffector, type).block(0, 0, 3, 6);
      for (size_t i = 0; i < chainSize; ++i) {
        if (!activeJoints[i]) {
          J.col(i).setZero();
        }
      }
      jInv = MathsUtils::pseudoInverse(J);
      result = jInv *  kP * diffPos;
    }
    for (size_t j = 0; j < chainSize; ++j) {
      auto index = chainStart + j;
      result[j] += JOINT_STATE(index, type)->position();
      if (abs(result[j]) > M_PI)
        result[j] = atan2(sin(result[j]), cos(result[j]));
      if (result[j] > joints[index]->maxPosition)
        result[j] = joints[index]->maxPosition;
      if (result[j] < joints[index]->minPosition)
        result[j] = joints[index]->minPosition;
    }
    this->setJointPositions(static_cast<Joints>(chainStart), result, type);
    prepareDHTransforms(chainIndex, type);
    initT = getForwardEffector(chainIndex, endEffector, type);
    initPos = initT.block(0, 3, 3, 1);
    if (solveForOrientation) {
      diffOrient = MathsUtils::getOrientationDiff(initT, targetT);
    }
    diffPos = targetPos - initPos;
    if (solveForOrientation) {
      if (diffPos.norm() < pTol && diffOrient.norm() < oTol) {
        success = true;
        Matrix<Scalar, Dynamic, 1> res =
          this->getJointPositions(static_cast<Joints>(chainStart), chainSize, type);
        setJointPositions(Joints::first, startJoints, startType);
        return res;
      } else {
        success = false;
      }
    } else {
      if (diffPos.norm() < pTol) {
        success = true;
        Matrix<Scalar, Dynamic, 1>
          res = this->getJointPositions(static_cast<Joints>(chainStart), chainSize, type);
        setJointPositions(Joints::first, startJoints, startType);
        return res;
      } else {
        success = false;
      }
    }
  }
  if (!success) {
    Matrix<Scalar, Dynamic, 1>
      res = this->getJointPositions(static_cast<Joints>(chainStart), chainSize, type);
    setJointPositions(Joints::first, startJoints, startType);
    return res;
  }
}

template <typename Scalar>
vector<Matrix<Scalar, Dynamic, 1> >
KinematicsModule<Scalar>::inverseLeftLeg(
  const Matrix<Scalar, 4, 4>& endEffector,
  const Matrix<Scalar, 4, 4>& targetT)
{
  auto type = JointStateType::sim;
  vector<Matrix<Scalar, Dynamic, 1> > returnResult;
  Matrix<Scalar, 4, 4> tTempTheta5, t4I, t5I, t6I, tTemp, tTemp2;
  Matrix<Scalar, 4, 4> t = targetT;
  Matrix<Scalar, 4, 4> tInit = t;

  ///<Move the start point to the hipyawpitch point
  Matrix<Scalar, 4, 4> base = tBaseLLegInv;
  base *= t;

  ///<Move the end point to the anklePitch joint
  base *= tEndLLegInv;

  ///<Rotate hipyawpitch joint
  Matrix<Scalar, 4, 4> rot = rotFixLLeg;
  rot *= base;

  ///<Invert the table, because we need the
  ///<chain from the ankle to the hip
  Matrix<Scalar, 4, 4> tStart = rot;
  rot = MathsUtils::getTInverse(rot);
  t = rot;

  ///<Build the rotation table
  Scalar side1 = thighLength;
  Scalar side2 = tibiaLength;
  Scalar distanceSqrd = pow(t.block(0, 3, 3, 1).norm(), 2);

  ///<Calculate Theta 4
  Scalar theta4 = M_PI - MathsUtils::safeAcos(
    (pow(side1, 2) + pow(side2, 2) - distanceSqrd) / (2 * side1 * side2));
  if (theta4 != theta4) {
    return returnResult;
  }
  Scalar theta6 = atan(t(1, 3) / t(2, 3));
  //if(theta6 < lAnkleRollLow || theta6 > lAnkleRollHigh)
  //  return returnResult;
  if (theta6 < lAnkleRollLow) theta6 = lAnkleRollLow;
  else if (theta6 > lAnkleRollHigh) theta6 = lAnkleRollHigh;

  MathsUtils::makeDHTransformation(
    t6I,
    (Scalar) 0.0,
    (Scalar) -M_PI_2,
    (Scalar) 0.0,
    theta6);
  t6I *= rotRLeg;
  //try
  //{
  t6I = MathsUtils::getTInverse(t6I);
  tStart *= t6I;
  tTempTheta5 = tStart;
  tTempTheta5 = MathsUtils::getTInverse(tTempTheta5);
  //  }
  //catch(KMath::KMat::SingularMatrixInvertionException d)
  //{
  //return returnResult;
  //}
  for (int itter = 0; itter < 2; itter++) {
    theta4 = (itter == 0) ? theta4 : -theta4;
    if (theta4 < rKneePitchLow || theta4 > rKneePitchHigh) continue;
    MathsUtils::makeDHTransformation(
      t4I,
      (Scalar) -thighLength,
      (Scalar) 0.0,
      (Scalar) 0.0,
      theta4);
    Scalar up =
      tTempTheta5(1, 3) * (tibiaLength + thighLength * cos(theta4)) + thighLength * tTempTheta5(
        0,
        3) * sin(theta4);
    Scalar down = pow(thighLength, 2) * pow(sin(theta4), 2) + pow(
      tibiaLength + thighLength * cos(theta4),
      2);
    Scalar theta5 = asin(-up / down);
    Scalar posOrNegPIt5 = (theta5 >= 0) ? M_PI : -M_PI;
    if (theta5 != theta5 && up / down < 0) theta5 = -M_PI_2;
    else if (theta5 != theta5) theta5 = M_PI_2;
    for (int i = 0; i < 2; i++) {
      if (i == 0 && (theta5 > lAnklePitchHigh || theta5 < lAnklePitchLow)) continue;
      else if (i == 1 && (posOrNegPIt5 - theta5 > lAnklePitchHigh || posOrNegPIt5 - theta5 < lAnklePitchLow)) continue;
      else if (i == 1) theta5 = posOrNegPIt5 - theta5;
      MathsUtils::makeDHTransformation(
        t5I,
        (Scalar) -tibiaLength,
        (Scalar) 0.0,
        (Scalar) 0.0,
        theta5);
      tTemp = t4I;
      tTemp *= t5I;
      //try
      //  {
      tTemp = MathsUtils::getTInverse(tTemp);
      //}
      //catch(KMath::KMat::SingularMatrixInvertionException d)
      //{
      //  continue;
      //}
      tTemp2 = tStart;
      tTemp2 *= tTemp;
      Scalar temptheta2 = MathsUtils::safeAcos(tTemp2(1, 2));
      Scalar theta2;
      for (int l = 0; l < 2; l++) {
        if (l == 0 && (temptheta2 - M_PI_4 > lHipRollHigh || temptheta2 - M_PI_4 < lHipRollLow)) continue;
        else if (l == 1 && (-temptheta2 - M_PI_4 > lHipRollHigh || -temptheta2 - M_PI_4 < lHipRollLow)) continue;
        else if (l == 0) theta2 = temptheta2 - M_PI_4;
        else if (l == 1) theta2 = -temptheta2 - M_PI_4;
        Scalar theta3 = asin(tTemp2(1, 1) / sin(theta2 + M_PI_4));
        Scalar posOrNegPIt3 = (theta3 >= 0) ? M_PI : -M_PI;
        if (theta3 != theta3 && tTemp2(1, 1) / sin(theta2 + M_PI_4) < 0) theta3 =
          -M_PI_2;
        else if (theta3 != theta3) theta3 = M_PI_2;
        for (int k = 0; k < 2; k++) {
          if (k == 0 && (theta3 > lHipPitchHigh || theta3 < lHipPitchLow)) continue;
          else if (k == 1 && (posOrNegPIt3 - theta3 > lHipPitchHigh || posOrNegPIt3 - theta3 < lHipPitchLow)) continue;
          else if (k == 1) theta3 = posOrNegPIt3 - theta3;
          Scalar temptheta1 = MathsUtils::safeAcos(
            tTemp2(0, 2) / sin(theta2 + M_PI_4));
          if (temptheta1 != temptheta1) temptheta1 = 0;
          for (int p = 0; p < 2; p++) {
            Scalar theta1;

            if (p == 0 && (temptheta1 + M_PI_2 > lHipYawPitchHigh || -temptheta1 + M_PI_2 < lHipYawPitchLow)) continue;
            else if (p == 1 && (-temptheta1 + M_PI_2 > lHipYawPitchHigh || -temptheta1 + M_PI_2 < lHipYawPitchLow)) continue;
            else if (p == 0) theta1 = temptheta1 + M_PI_2;
            else if (p == 1) theta1 = -temptheta1 + M_PI_2;

            ///<Forward VALID step
            JOINT_STATE(Joints::lHipYawPitch, type)->setPosition(theta1);
            JOINT_STATE(Joints::lHipRoll, type)->setPosition(theta2);
            JOINT_STATE(Joints::lHipPitch, type)->setPosition(theta3);
            JOINT_STATE(Joints::lKneePitch, type)->setPosition(theta4);
            JOINT_STATE(Joints::lAnklePitch, type)->setPosition(theta5);
            JOINT_STATE(Joints::lAnkleRoll, type)->setPosition(theta6);
            prepareDHTransforms(LinkChains::lLeg, type);
            Matrix<Scalar, 4, 4> test = getForwardEffector(LinkChains::lLeg, endEffector, type);
            if (MathsUtils::almostEqual(test, tInit)) {
              Matrix<Scalar, Dynamic, 1> r(toUType(HardwareIds::nRLeg));
              r[0] = theta1;
              r[1] = theta2;
              r[2] = theta3;
              r[3] = theta4;
              r[4] = theta5;
              r[5] = theta6;
              returnResult.push_back(r);
            }
          }
        }
      }
    }
  }
  return returnResult;
}

template <typename Scalar>
vector<Matrix<Scalar, Dynamic, 1> >
KinematicsModule<Scalar>::inverseRightLeg(const Matrix<Scalar, 4, 4>& endEffector,
  const Matrix<Scalar, 4, 4>& targetT)
{
  Matrix<Scalar, 4, 4> mirrored = MathsUtils::mirrorTransformation(targetT);
  vector<Matrix<Scalar, Dynamic, 1> > res = inverseLeftLeg(endEffector, mirrored);
  for (size_t i = 0; i < res.size(); i++) {
    res[i][1] = -res[i][1]; //HIP_ROLL
    res[i][5] = -res[i][5]; //ANKLE_ROLL
  }
  return res;
}

template <typename Scalar>
void KinematicsModule<Scalar>::setStateFromTo(
  const JointStateType& from,
  const JointStateType& to)
{
  for (size_t i = 0; i < joints.size(); ++i) {
    *joints[i]->states[(unsigned)to] =
      *joints[i]->states[(unsigned)from];
    joints[i]->states[(unsigned)to]->setPosition(joints[i]->states[(unsigned)from]->position());
  }
  bodyToGlobal[(unsigned)to] = getForwardEffector(static_cast<LinkChains>(globalBase), toUType(globalEnd), to);
  globalToBody[(unsigned)to] = MathsUtils::getTInverse(bodyToGlobal[(unsigned)to]);
  Matrix<Scalar, 3, 3> rot = globalToBody[(unsigned)to].block(0, 0, 3, 3);
  globalToBodyRotX[(unsigned)to].block(0, 0, 3, 3) = rot;
  globalToBodyRotX[(unsigned)to].block(3, 3, 3, 3) = rot;
}

template <typename Scalar>
void KinematicsModule<Scalar>::setJointPositions(
  const Joints& startIndex,
  const Matrix<Scalar, Dynamic, 1>& simPosition,
  const JointStateType& type,
  const bool& solveFk)
{
  ASSERT(toUType(startIndex) + simPosition.size() <= toUType(Joints::count));
  for (size_t i = 0; i < simPosition.size(); ++i) {
    joints[toUType(startIndex) + i]->states[toUType(type)]->setPosition(simPosition[i]);
  }
  prepareDHTransforms(LinkChains::count, type, solveFk);
}

template <typename Scalar>
void KinematicsModule<Scalar>::setChainPositions(
  const LinkChains& chainIndex,
  const Matrix<Scalar, Dynamic, 1>& simPosition,
  const JointStateType& type,
  const bool& solveFk)
{
  unsigned size = linkChains[toUType(chainIndex)]->size;
  ASSERT(simPosition.size() == size);
  for (size_t i = 0; i < simPosition.size(); ++i) {
    joints[linkChains[toUType(chainIndex)]->start + i]->states[toUType(type)]->setPosition(simPosition[i]);
  }
  prepareDHTransforms(chainIndex, type, solveFk);
}

template <typename Scalar>
void KinematicsModule<Scalar>::setJointVelocities(
  const Joints& startIndex,
  const Matrix<Scalar, Dynamic, 1>& simVelocities,
  const JointStateType& type,
  const bool& solveFk)
{
  ASSERT(toUType(startIndex) + simVelocities.size() <= toUType(Joints::count));
  for (size_t i = 0; i < simVelocities.size(); ++i) {
    joints[toUType(startIndex) + i]->states[toUType(type)]->setVelocity(simVelocities[i]);
  }
  prepareDHTransforms(LinkChains::count, type, solveFk);
}

template <typename Scalar>
void KinematicsModule<Scalar>::setChainVelocities(
  const LinkChains& chainIndex,
  const Matrix<Scalar, Dynamic, 1>& simVelocities,
  const JointStateType& type,
  const bool& solveFk)
{
  unsigned size = linkChains[toUType(chainIndex)]->size;
  ASSERT(simVelocities.size() == size);
  for (size_t i = 0; i < simVelocities.size(); ++i) {
    joints[linkChains[toUType(chainIndex)]->start + i]->
      states[toUType(type)]->setVelocity(simVelocities[i]);
  }
  prepareDHTransforms(chainIndex, type, solveFk);
}

template <typename Scalar>
void KinematicsModule<Scalar>::setJointAccelerations(
  const Joints& startIndex,
  const Matrix<Scalar, Dynamic, 1>& simAccelerations,
  const JointStateType& type,
  const bool& solveFk)
{
  ASSERT(toUType(startIndex) + simAccelerations.size() <= toUType(Joints::count));
  for (size_t i = 0; i < simAccelerations.size(); ++i) {
    joints[toUType(startIndex) + i]->states[toUType(type)]->
      setAccel(simAccelerations[i]);
  }
  prepareDHTransforms(LinkChains::count, type, solveFk);
}

template <typename Scalar>
void KinematicsModule<Scalar>::setChainAccelerations(
  const LinkChains& chainIndex,
  const Matrix<Scalar, Dynamic, 1>& simAccelerations,
  const JointStateType& type,
  const bool& solveFk)
{
  unsigned size = linkChains[toUType(chainIndex)]->size;
  ASSERT(simAccelerations.size() == size);
  for (size_t i = 0; i < simAccelerations.size(); ++i) {
    joints[linkChains[toUType(chainIndex)]->start + i]->states[toUType(type)]->
      setAccel(simAccelerations[i]);
  }
  prepareDHTransforms(chainIndex, type, solveFk);
}

template <typename Scalar>
void KinematicsModule<Scalar>::setJointState(
  const Joints& startIndex,
  const Matrix<Scalar, Dynamic, 1>& simPosition,
  const Matrix<Scalar, Dynamic, 1>& simVelocity,
  const Matrix<Scalar, Dynamic, 1>& simAcceleration,
  const JointStateType& type,
  const bool& solveFk)
{
  ASSERT(toUType(startIndex) + simPosition.size() <= toUType(Joints::count));
  ASSERT(
    simPosition.size() == simVelocity.size() &&
    simPosition.size() == simAcceleration.size());
  for (size_t i = 0; i < simPosition.size(); ++i) {
    joints[toUType(startIndex) + i]->states[toUType(type)]->setPosition(simPosition[i]);
    joints[toUType(startIndex) + i]->states[toUType(type)]->setVelocity(simVelocity[i]);
    joints[toUType(startIndex) + i]->states[toUType(type)]->setAccel(simAcceleration[i]);
  }
  prepareDHTransforms(LinkChains::count, type, solveFk);
}

template <typename Scalar>
void KinematicsModule<Scalar>::setChainState(
  const LinkChains& chainIndex,
  const Matrix<Scalar, Dynamic, 1>& simPosition,
  const Matrix<Scalar, Dynamic, 1>& simVelocity,
  const Matrix<Scalar, Dynamic, 1>& simAcceleration,
  const JointStateType& type,
  const bool& solveFk)
{
  unsigned size = linkChains[toUType(chainIndex)]->size;
  ASSERT(simPosition.size() == size);
  ASSERT(
    simPosition.size() == simVelocity.size() &&
    simPosition.size() == simAcceleration.size());
  for (size_t i = 0; i < simPosition.size(); ++i) {
    joints[linkChains[toUType(chainIndex)]->start + i]->
      states[toUType(type)]->setPosition(simPosition[i]);
    joints[linkChains[toUType(chainIndex)]->start + i]->
      states[toUType(type)]->setVelocity(simVelocity[i]);
    joints[linkChains[toUType(chainIndex)]->start + i]->
      states[toUType(type)]->setAccel(simAcceleration[i]);
  }
  /*cout << "Pos: " << endl;
  cout << "simPosition: " << simPosition << endl;
  cout << "pos: " << endl;
  for (size_t i = 0; i < simPosition.size(); ++i) {
    cout << JOINT_STATE(linkChains[chainIndex]->start + i, type)->position() << " ";
  }
  cout << endl;
  cout << "Vel: " << endl;
  cout << "simVelocity: " << simVelocity << endl;
  for (size_t i = 0; i < simPosition.size(); ++i) {
    cout << JOINT_STATE(linkChains[chainIndex]->start + i, type)->velocity() << " ";
  }
  cout << endl;
  cout << "Acc: " << endl;
  cout << "simAcceleration: " << simAcceleration << endl;
  for (size_t i = 0; i < simPosition.size(); ++i) {
    cout << JOINT_STATE(linkChains[chainIndex]->start + i, type)->accel() << " ";
  }
  cout << endl;*/
  prepareDHTransforms(LinkChains::count, type, solveFk);
}

template <typename Scalar>
void KinematicsModule<Scalar>::setJointPositionCmd(
  const Matrix<Scalar, Dynamic, 1>& cmd)
{
  //auto& posSensors = JOINT_POSITIONS_OUT(MotionModule);
  for (size_t i = 0; i < toUType(Joints::count); ++i) {
    if (cmd[i] != cmd[i]) // NAN
      JOINT_STATE_ACTUAL(i)->estimator()->setInput(0.0);
    else
      JOINT_STATE_ACTUAL(i)->estimator()->setControl(cmd[i]);
  }
}

template <typename Scalar>
boost::shared_ptr<Joint<Scalar> >
KinematicsModule<Scalar>::getJoint(const Joints& index)
{
  return joints[toUType(index)];
}

template <typename Scalar>
boost::shared_ptr<JointState<Scalar> >
KinematicsModule<Scalar>::getJointState(
  const Joints& index,
  const JointStateType& type)
{
  return joints[toUType(index)]->states[toUType(type)];
}

template <typename Scalar>
vector<boost::shared_ptr<Joint<Scalar> > >
KinematicsModule<Scalar>::getJoints(
  const Joints& startIndex,
  const unsigned& nElements)
{
  ASSERT(toUType(startIndex) + nElements <= toUType(Joints::count));
  return
    vector<boost::shared_ptr<Joint<Scalar> > >(
      joints.begin() + toUType(startIndex),
      joints.begin() + toUType(startIndex) + nElements
    );
}

template <typename Scalar>
vector<boost::shared_ptr<JointState<Scalar> > >
KinematicsModule<Scalar>::getJointStates(
  const Joints& startIndex,
  const unsigned& nElements,
  const JointStateType& type)
{
  ASSERT(toUType(startIndex) + nElements <= toUType(Joints::count));
  vector<boost::shared_ptr<JointState<Scalar> > > states;
  for (size_t i = toUType(startIndex); i < toUType(startIndex) + nElements; ++i)
    states.push_back(joints[i]->states[toUType(type)]);
  return states;
}

template <typename Scalar>
vector<boost::shared_ptr<JointState<Scalar> > >
KinematicsModule<Scalar>::getChainStates(
  const LinkChains& chainIndex,
  const JointStateType& type)
{
  unsigned start = linkChains[toUType(chainIndex)]->start;
  unsigned size = linkChains[toUType(chainIndex)]->size;
  vector<boost::shared_ptr<JointState<Scalar> > > states;
  for (size_t i = start; i < start + size; ++i)
    states.push_back(joints[i]->states[toUType(type)]);
  return states;
}
template <typename Scalar>
Scalar KinematicsModule<Scalar>::getJointPosition(
  const Joints& index,
  const JointStateType& type)
{
  return joints[toUType(index)]->states[toUType(type)]->position() + joints[toUType(index)]->states[toUType(type)]->offset;
}

template <typename Scalar>
Matrix<Scalar, Dynamic, 1> KinematicsModule<Scalar>::getJointPositions(
  const Joints& startIndex,
  const unsigned& nElements,
  const JointStateType& type)
{
  Matrix<Scalar, Dynamic, 1> positions;
  positions.resize(nElements);
  for (size_t i = 0; i < nElements; ++i) {
    positions[i] = joints[toUType(startIndex) + i]->states[toUType(type)]->position() + joints[toUType(startIndex) + i]->states[toUType(type)]->offset;
  }
  return positions;
}

template <typename Scalar>
Matrix<Scalar, Dynamic, 1> KinematicsModule<Scalar>::getJointVelocities(
  const Joints& startIndex,
  const unsigned& nElements,
  const JointStateType& type)
{
  Matrix<Scalar, Dynamic, 1> velocities;
  velocities.resize(nElements);
  for (size_t i = 0; i < nElements; ++i) {
    velocities[i] = joints[toUType(startIndex) + i]->states[toUType(type)]->velocity();
  }
  return velocities;
}

template <typename Scalar>
boost::shared_ptr<LinkInfo<Scalar> >
KinematicsModule<Scalar>::getLink(const Links& index)
{
  return links[toUType(index)];
}

template <typename Scalar>
boost::shared_ptr<LinkChain<Scalar> >
KinematicsModule<Scalar>::getLinkChain(const LinkChains& index)
{
  return linkChains[toUType(index)];
}

template <typename Scalar>
RobotFeet KinematicsModule<Scalar>::getGlobalBaseIndex()
{
  return globalBase;
}

template <typename Scalar>
boost::shared_ptr<LinkChain<Scalar> >
KinematicsModule<Scalar>::getGlobalBase()
{
  return linkChains[toUType(globalBase)];
}

template <typename Scalar>
Matrix<Scalar, 4, 4>  KinematicsModule<Scalar>::getEndEffector(
  const LinkChains& chain, const unsigned& index)
{
  return linkChains[toUType(chain)]->endEffectors[index];
}

template <typename Scalar>
boost::shared_ptr<TorsoState<Scalar> >
KinematicsModule<Scalar>::getTorsoState()
{
  return torsoState;
}

template <typename Scalar> Matrix<Scalar, 4, 4>
KinematicsModule<Scalar>::getGlobalToBody(const JointStateType& type)
{
  return globalToBody[toUType(type)];
}

template <typename Scalar> Matrix<Scalar, 4, 4>
KinematicsModule<Scalar>::getBodyToGlobal(const JointStateType& type)
{
  return bodyToGlobal[toUType(type)];
}

template <typename Scalar>
void KinematicsModule<Scalar>::setGlobalBase(
  const RobotFeet& globalBase, const LegEEs& globalEnd)
{
  if (globalBase != this->globalBase) {
    LOG_INFO("Setting new global base...");
    ///< Get the transformation from new globalBase to previous globalBase
    Matrix<Scalar, 4, 4> trans =
      MathsUtils::getTInverse(
        (Matrix<Scalar, 4, 4>)(getGlobalToBody() *
        getForwardEffector(static_cast<LinkChains>(globalBase), toUType(globalEnd))));
    this->globalBase = globalBase;
    this->globalEnd = globalEnd;
    prepareDHTransforms();

    comState->position = MathsUtils::transformVector(trans, comState->position);
    comState->velocity = trans.block(0, 0, 3, 3) * comState->velocity;
    comState->accel = trans.block(0, 0, 3, 3) * comState->accel;
    Matrix<Scalar, 3, 1> xState, yState;
    xState << comState->position[0], comState->velocity[0], comState->accel[0];
    yState << comState->position[1], comState->velocity[1], comState->accel[1];
    comEstimator[0]->setState(xState);
    comEstimator[1]->setState(yState);
    comState->zmp[0] = comEstimator[0]->getOutput()[0];
    comState->zmp[1] = comEstimator[1]->getOutput()[0];
    comState->baseFrame = globalBase;
    comState->eeIndex = toUType(globalEnd);
  }
}

template <typename Scalar>
RobotFeet KinematicsModule<Scalar>::getFootOnGround()
{
  return footOnGround;
}

template <typename Scalar>
Scalar KinematicsModule<Scalar>::getFootSpacing()
{
  return footSpacing;
}

template <typename Scalar>
Scalar KinematicsModule<Scalar>::getCycleTime()
{
  return cycleTime;
}

template <typename Scalar>
void KinematicsModule<Scalar>::setJointOffset(const Joints& index, const Scalar& offset, const JointStateType& type) {
  joints[toUType(index)]->states[toUType(type)]->offset = offset;
}

template class KinematicsModule<MType>;

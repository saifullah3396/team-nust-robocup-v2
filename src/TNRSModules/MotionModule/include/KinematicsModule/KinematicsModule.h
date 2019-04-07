/**
 * @file MotionModule/include/KinematicsModule/KinematicsModule.h
 *
 * This file declares the class for solving the kinematics
 * of the robot.
 *
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

#pragma once

#include <fstream>
#include <boost/circular_buffer.hpp>
#include <boost/shared_ptr.hpp>
#include "TNRSBase/include/MemoryBase.h"
#include "MotionModule/include/MTypeHeader.h"
#include "MotionModule/include/KinematicsModule/JointStateType.h"
#include "Utils/include/MathsUtils.h"
#include "Utils/include/HardwareIds.h"

#ifdef MODULE_IS_REMOTE
//#define ALLOW_SYMBOLIC_COMPUTATIONS
#endif

class MotionModule;
template <typename Scalar, size_t MeasSize>
class ComEstimator;
template <typename Scalar, size_t StateSize, size_t InputSize, size_t OutputSize>
class ProcessModel;
template <typename Scalar>
class ComState;
template <typename Scalar>
class TorsoState;
template <typename Scalar>
class ImuFilter;
template <typename Scalar>
class Joint;
template <typename Scalar>
class JointState;
template <typename Scalar>
class LinkInfo;
template <typename Scalar>
class LinkChain;
template <typename Scalar>
class MotionTask;
template <typename Scalar>
class PostureTask;
template <typename Scalar>
class ContactTask;
template <typename Scalar>
class CartesianTask;
template <typename Scalar>
class ComTask;
template <typename Scalar>
class TorsoTask;
template <typename Scalar>
class TaskIkSolver;
namespace Utils{
  class JsonLogger;
}
typedef boost::shared_ptr<Utils::JsonLogger> JsonLoggerPtr;
#ifdef ALLOW_SYMBOLIC_COMPUTATIONS
namespace SymEngine {
  class DenseMatrix;
}
#endif

/**
 * @class KinematicsModule
 * @brief The class for kinematics and dynamics of the robot.
 */
template <typename Scalar>
class KinematicsModule : public MemoryBase
{
public:
  /**
   * Default constructor for this class
   *
   * @param motionModule: base class.
   */
  KinematicsModule(MotionModule* motionModule);

  /**
   * Default destructor fr this class
   */
  ~KinematicsModule();

  /**
   * Sets up kinematic model of the robot by defining the
   * joints, links, and link chains
   */
  void init();

  /**
   * Updates the kinematics model of the robot based
   * on position sensor data.
   */
  void update();

  /**
   * @brief cleanup Performs clean up operations
   */
  void cleanup();

  /**
   * Sets the simulated joint states equal to actual joint state
   */
  void setStateFromTo(
    const JointStateType& from,
    const JointStateType& to);

  /**
   * Sets the simulated joint positions for simulated calculations.
   *
   * @param startIndex: First joint index.
   * @param simPosition: Simulated joint positions.
   * @param type: Type of joints.
   */
  void setJointPositions(
    const Joints& startIndex,
    const Matrix<Scalar, Dynamic, 1>& simPosition,
    const JointStateType& type = JointStateType::actual,
    const bool& solveFk = true);

  /**
   * Sets the simulated joint positions for simulated calculations.
   *
   * @param chainIndex: Chain to update the joint values of
   * @param simPosition: Simulated joint positions.
   * @param type: Type of joints.
   */
  void setChainPositions(
    const LinkChains& chainIndex,
    const Matrix<Scalar, Dynamic, 1>& simPosition,
    const JointStateType& type = JointStateType::actual,
    const bool& solveFk = true);

  /**
   * Sets the simulated joint velocities for simulated calculations.
   *
   * @param startIndex: First joint index.
   * @param simVelocities: Simulated joint velocities.
   * @param type: Type of joints
   */
  void setJointVelocities(
    const Joints& startIndex,
    const Matrix<Scalar, Dynamic, 1>& simVelocities,
    const JointStateType& type = JointStateType::actual,
    const bool& solveFk = true);

  /**
   * Sets the simulated joint velocities for simulated calculations.
   *
   * @param chainIndex: Chain to update the joint values of
   * @param simVelocities: Simulated joint velocities.
   * @param type: Type of joints
   */
  void setChainVelocities(
    const LinkChains& chainIndex,
    const Matrix<Scalar, Dynamic, 1>& simVelocities,
    const JointStateType& type = JointStateType::actual,
    const bool& solveFk = true);

  /**
   * Sets the simulated joint accelerations for simulated calculations.
   *
   * @param startIndex: First joint index.
   * @param simAccelerations: Simulated joint Accelerations.
   * @param type: Type of joints
   */
  void setJointAccelerations(
    const Joints& startIndex,
    const Matrix<Scalar, Dynamic, 1>& simAccelerations,
    const JointStateType& type = JointStateType::actual,
    const bool& solveFk = true);

  /**
   * Sets the simulated joint accelerations for simulated calculations.
   *
   * @param chainIndex: Chain to update the joint values of
   * @param simAccelerations: Simulated joint Accelerations.
   * @param type: Type of joints
   */
  void setChainAccelerations(
    const LinkChains& chainIndex,
    const Matrix<Scalar, Dynamic, 1>& simAccelerations,
    const JointStateType& type = JointStateType::actual,
    const bool& solveFk = true);

  /**
   * Sets the simulated joint states for simulated calculations.
   *
   * @param startIndex: First joint index.
   * @param simPosition: Simulated joint positions.
   * @param simVelocity: Simulated joint positions.
   * @param simAcceleration: Simulated joint positions.
   * @param type: Type of joints
   */
  void setJointState(
    const Joints& startIndex,
    const Matrix<Scalar, Dynamic, 1>& simPosition,
    const Matrix<Scalar, Dynamic, 1>& simVelocity,
    const Matrix<Scalar, Dynamic, 1>& simAcceleration,
    const JointStateType& type = JointStateType::actual,
    const bool& solveFk = true);

  /**
   * Sets the simulated joint states for simulated calculations.
   *
   * @param chainIndex: Chain to update the joint values of
   * @param simPosition: Simulated joint positions.
   * @param simVelocity: Simulated joint positions.
   * @param simAcceleration: Simulated joint positions.
   * @param type: Type of joints
   */
  void setChainState(
    const LinkChains& chainIndex,
    const Matrix<Scalar, Dynamic, 1>& simPosition,
    const Matrix<Scalar, Dynamic, 1>& simVelocity,
    const Matrix<Scalar, Dynamic, 1>& simAcceleration,
    const JointStateType& type = JointStateType::actual,
    const bool& solveFk = true);

  /**
   * @brief setJointPositionControl Sets the commanded joint values as input to
   *   joint estimator
   * @param input Commanded joints input vector
   */
  void setJointPositionCmd(
    const Matrix<Scalar, Dynamic, 1>& cmd);

  /**
   * Converts the specified cartesian velocities to joint velocities
   * for the given chain.
   *
   * @param chainIndex: Index of the limb.
   * @param cVels: The cartesian space velocity vector.
   * @param endEffector: The end effector transformation
   *   matrix from the final frame of the chain
   * @param type: Type of joints
   *
   * @return Matrix<Scalar, Dynamic, 1>
   */
  Matrix<Scalar, Dynamic, 1> cartToJointVels(const LinkChains& chainIndex,
    const Matrix<Scalar, Dynamic, 1> cVels, const Matrix<Scalar, 4, 4> endEffector,
    const JointStateType& type = JointStateType::actual);

  /**
   * Finds the jacobian for global base chain
   *
   * @param type: Type of joints
   *
   * @return Matrix<Scalar, Dynamic, Dynamic>
   */
  Matrix<Scalar, 6, Dynamic> computeGlobalBaseJ(
    const JointStateType& type);
  /**
   * Finds the specified limb jacobian with respect to a given
   * end-effector vector.
   *
   * @param chainIndex: Index of the limb.
   * @param eeIndex: The end effector index.
   * @param type: Type of joints
   *
   * @return Matrix<Scalar, Dynamic, Dynamic>
   */
  Matrix<Scalar, 6, Dynamic> computeLimbJ(const LinkChains& chainIndex,
    const unsigned& eeIndex,
    const JointStateType& type = JointStateType::actual,
    const bool& relGlobalBase = true);

  /**
   * Finds the specified limb jacobian with respect to a given
   * end-effector vector.
   *
   * @param chainIndex: Index of the limb.
   * @param endEffector: The end effector transformation
   *   matrix from the final frame of the chain
   * @param type: Type of joints
   *
   * @return Matrix<Scalar, Dynamic, Dynamic>
   */
  Matrix<Scalar, 6, Dynamic> computeLimbJ(const LinkChains& chainIndex,
    const Matrix<Scalar, 4, 4>& endEffector,
    const JointStateType& type = JointStateType::actual,
    const bool& relGlobalBase = true);

  /**
   * Finds the Center of mass jacobian for the whole body
   * @param type: Type of joints
   *
   * @return Matrix<Scalar, Dynamic, Dynamic>
   */
  Matrix<Scalar, 3, Dynamic> computeComJacobian(
    const JointStateType& type = JointStateType::actual);

  /**
   * Finds the specified limb Center of mass jacobian with respect to
   * the base of the limb.
   *
   * @param chainIndex: Index of the limb.
   * @param type: Type of joints
   *
   * @return Matrix<Scalar, Dynamic, Dynamic>
   */
  Matrix<Scalar, 3, Dynamic> computeLimbComJ(
    const LinkChains& chainIndex,
    const JointStateType& type = JointStateType::actual);

  /**
   * Finds the center of mass jacobian for the given link
   *
   * @param index: Link index
   * @param jacobianV: Com velocity jacobian to be updated
   * @param jacobianW: Com angular velocity jacobian to be updated
   * @param type: Type of joints
   *
   * @return void
   */
  void computeLinkComJ(
    const Links& index,
    Matrix<Scalar, 3, Dynamic>& jacobianV,
    Matrix<Scalar, 3, Dynamic>& jacobianW,
    const JointStateType& type = JointStateType::actual);

  /**
   * Solves the inverse kinematics for the center of mass X-Y while
   * keeping the feet constrained on the ground.
   *
   * @param desCom: Desired center of mass position, only xy matters here
   * @param baseLimb: Support limb
   * @param eeIndex: Index of the support foot end-effector
   * @param type: Type of joints
   *
   * @return Joint values of the whole body
   */
  Matrix<Scalar, Dynamic, 1> solveComIkTwoVar(
    const Matrix<Scalar, 3, 1>& desCom,
    const LinkChains& LinkChains,
    const unsigned& eeIndices,
    const JointStateType& type);

  /**
   * Solves the inverse kinematics for the center of mass given the
   * required base limb to move it.
   *
   * @param baseLimb: Index of the support limb.
   * @param comVelocityD: Desired Com velocity.
   * @param limbMotionSpace: Desired limbs motion type, that is,
   *   cartesian or joint space.
   * @param limbVelocitiesD: Desired limb joint/cartesian space
   *   velocities depending upon the motion space.
   * @param eeIndices: Indices of desired end effectors for each limb.
   * @param type: Type of joints
   *
   * @return Matrix<Scalar, Dynamic, 1>: Ik values of whole body joints.
   */
  Matrix<Scalar, Dynamic, 1> solveComIK(const LinkChains& baseLimb,
    const Matrix<Scalar, 6, 1>& comVelocityD,
    const vector<unsigned>& limbMotionSpace,
    const vector<Matrix<Scalar, Dynamic, 1>>& limbVelocitiesD,
    const vector<int>& eeIndices,
    const JointStateType& type = JointStateType::actual);

  /**
   * Solves the inverse kinematics for the given chain and end-effector
   * using pseudo inverse of jacobian.
   *
   * @param chainIndex: Index of the end-effector chain.
   * @param eeIndex: The end effector transformation index.
   * @param targetT: The desired configuration of the end effector.
   * @param startType: Type of joints to find the initial position of
   *   end-effector.
   *
   * @return Matrix<Scalar, Dynamic, 1>: Ik values of the chain joints.
   */
  Matrix<Scalar, Dynamic, 1> solveJacobianIK(const LinkChains& chainIndex,
    const unsigned& eeIndex, const Matrix<Scalar, 4, 4>& targetT,
    const unsigned& maxIterations = 100,
    const JointStateType& startType = JointStateType::actual, const bool& solveForOrientation = true,
    const Scalar& pTol = 1e-2,
    const Scalar& oTol = 0.5,
    vector<bool>& activeJoints = vector<bool>());

  /**
   * Makes a posture task for the given active joints
   *
   * @param targetJoints: Target joints for the body
   *
   * @return PostureTaskPtr
   */
  boost::shared_ptr<PostureTask<Scalar> > makePostureTask(
    const Matrix<Scalar, Dynamic, 1>& targetJoints,
    vector<bool> activeJoints = vector<bool>(),
    const Scalar& weight = 1e-6,
    const Scalar& gain = 0.85,
    vector<bool> activeResidual = vector<bool>()
  );

  /**
   * Makes a contact task for the given chain and end-effector
   *
   * @param chainIndex: Chain index
   * @param eeIndex: End-effector index
   *
   * @return ContactTaskPtr
   */
  boost::shared_ptr<ContactTask<Scalar> > makeContactTask(
    const LinkChains& chainIndex,
    const unsigned& eeIndex,
    vector<bool> activeJoints = vector<bool>(),
    const Scalar& weight = 10,
    const Scalar& gain = 0.9,
    vector<bool> activeResidual = vector<bool>()
  );

  /**
   * Makes a cartesian task for the given chain and end-effector
   *
   * @param chainIndex: Chain index
   * @param ee: End-effector
   * @param targetT: Target transformation for the end-effector
   *
   * @return CartesianTaskPtr
   */
  boost::shared_ptr<CartesianTask<Scalar> > makeCartesianTask(
    const LinkChains& chainIndex,
    const Matrix<Scalar, 4, 4>& ee,
    const Matrix<Scalar, 4, 4>& targetT,
    vector<bool> activeJoints = vector<bool>(),
    const Scalar& weight = 1,
    const Scalar& gain = 0.9,
    vector<bool> activeResidual = vector<bool>()
  );

  /**
   * Makes a cartesian task for the given chain and end-effector
   *
   * @param chainIndex: Chain index
   * @param eeIndex: End-effector index
   * @param targetT: Target transformation for the end-effector
   *
   * @return CartesianTaskPtr
   */
  boost::shared_ptr<CartesianTask<Scalar> > makeCartesianTask(
    const LinkChains& chainIndex,
    const unsigned& eeIndex,
    const Matrix<Scalar, 4, 4>& targetT,
    vector<bool> activeJoints = vector<bool>(),
    const Scalar& weight = 1,
    const Scalar& gain = 0.9,
    vector<bool> activeResidual = vector<bool>()
  );

  /**
   * @brief makeComTask Makes a com task for the given active joints
   * @param baseFrame Base frame
   * @param baseEE Base frame end-effector
   * @param comTarget Target Com position
   * @param activeJoints Active joints
   * @param weight Task weight
   * @param gain Task gain
   * @param activeResidual Active residual for this task
   * @return ComTask
   */
  boost::shared_ptr<ComTask<Scalar> > makeComTask(
    // com reference frame (left or right foot defined by CHAIN_L_LEG or CHAIN_R_LEG)
    const RobotFeet& baseFrame,
    const LegEEs& baseEE,
    const Matrix<Scalar, 3, 1>& comTarget,
    vector<bool> activeJoints = vector<bool>(),
    const Scalar& weight = 1,
    const Scalar& gain = 0.9,
    vector<bool> activeResidual = vector<bool>()
  );

  /**
   * @brief makeTorsoTask Makes a torso task for the given active joints
   * @param baseFrame Base frame
   * @param baseEE Base frame end-effector
   * @param target Target torso transformation
   * @param activeJoints Active joints
   * @param weight Task weight
   * @param gain Task gain
   * @param activeResidual Active residual for this task
   * @return ComTask
   */
  boost::shared_ptr<TorsoTask<Scalar> > makeTorsoTask(
    const RobotFeet& baseFrame,
    const LegEEs& baseEE,
    const Matrix<Scalar, 4, 4>& target,
    vector<bool> activeJoints,
    const Scalar& weight,
    const Scalar& gain,
    vector<bool> activeResidual = vector<bool>());

  /**
   * @brief solveCartesianIK: Solves the inverse kinematics for the
   *   given cartesian task using taskIkSolver
   * @param chainIndex: Index of the cartesian chain
   * @param eeIndex: Index of the end-effector
   * @param targetT: Target transformation for the end-effector
   * @param maxIterations: Max number of iterations to solve ik
   * @return ik-solved joint values
   */
  Matrix<Scalar, Dynamic, 1> solveCartesianIK(
    const LinkChains& chainIndex,
    const unsigned& eeIndex,
    const Matrix<Scalar, 4, 4>& targetT,
    const unsigned& maxIterations = 100);

  /**
   * @brief solveTasksIK: Solves the inverse kinematics for the
   *   all the given weighted tasks using taskIkSolver
   * @param tasks: Tasks
   * @param maxIterations: Max number of iterations to solve ik
   * @return ik-solved joint values
   */
  Matrix<Scalar, Dynamic, 1> solveTasksIK(
    const vector<boost::shared_ptr<MotionTask<Scalar> > >& tasks,
    const unsigned& maxIterations);

  /**
   * Solves the inverse kinematics for the given chain and end-effector
   * using pseudo inverse of jacobian.
   *
   * @param chainIndex: Index of the end-effector chain.
   * @param endEffector: The end effector transformation.
   * @param targetT: The desired configuration of the end effector.
   * @param startType: Type of joints to find the initial position of
   *   end-effector.
   *
   * @return Matrix<Scalar, Dynamic, 1>: Ik values of the chain joints.
   */
  Matrix<Scalar, Dynamic, 1> solveJacobianIK(const LinkChains& chainIndex,
    const Matrix<Scalar, 4, 4>& endEffector, const Matrix<Scalar, 4, 4>& targetT,
    const unsigned& maxIterations = 100,
    const JointStateType& startType = JointStateType::actual, const bool& solveForOrientation = true,
    const Scalar& pTol = 1e-2,
    const Scalar& oTol = 0.5,
    vector<bool>& activeJoints = vector<bool>());

  Matrix<Scalar, Dynamic, 1> solveJacobianIK(
    const LinkChains& chainIndex,
    const Matrix<Scalar, 4, 4>& endEffector,
    const Matrix<Scalar, 4, 4>& targetT,
    const unsigned& maxIterations,
    vector<bool>& activeJoints,
    vector<bool> activeResidual = vector<bool>());

  /**
   * Finds the specified limb mass matrix with respect to
   * the base of the limb.
   *
   * @param chainIndex: Index of the limb.
   * @param type: Type of joints
   *
   * @return Matrix<Scalar, Dynamic, Dynamic>
   */
  Matrix<Scalar, Dynamic, Dynamic> computeMassMatrix(
    const LinkChains& chainIndex,
    const JointStateType& type = JointStateType::actual);

  /**
   * Finds the specified limb joint torques by performing newton euler
   * recursion with given external force and torques.
   *
   * @param chainIndex: Index of the limb.
   * @param extForces: External forces.
   * @param extMoments: External moments.
   * @param totalForces: Container for joint forces.
   * @param totalMoments: Container for joint forces.
   * @param supportLeg: The leg base to be used as the inertial frame
   * @param type: Type of joints
   *
   * @return Matrix<Scalar, Dynamic, 1> returns the joint torques
   */
  Matrix<Scalar, Dynamic, 1> newtonEulerForces(const LinkChains& chainIndex,
    const Matrix<Scalar, 3, 1>& extForces, const Matrix<Scalar, 3, 1>& extMoments,
    Matrix<Scalar, 3, 1>& totalForces, Matrix<Scalar, 3, 1>& totalMoments,
    const LinkChains& supportLeg,
    const JointStateType& type = JointStateType::actual);

#ifdef ALLOW_SYMBOLIC_COMPUTATIONS
  SymEngine::DenseMatrix newtonEulerForcesSym(
    const unsigned& chainIndex,
    const SymEngine::DenseMatrix& extForces,
    const SymEngine::DenseMatrix& extMoments,
    SymEngine::DenseMatrix& totalForces,
    SymEngine::DenseMatrix& totalMoments,
    const unsigned& supportLeg);
#endif

  /**
   * Computes the Zmp of the robot by solving the system dynamics using
   * Newton Euler approach.
   *
   * @param supportLeg: Index of the support leg.
   * @param eeIndex: The end effector index.
   * @param type: Type of joints
   *
   * @return Matrix<Scalar, 2, 1>
   */
  Matrix<Scalar, 2, 1> computeZmp(
    const LinkChains& supportLeg,
    const JointStateType& type = JointStateType::actual
  );

  /**
   * Computes the Zmp of the robot for given forces and moments acting on torso
   *
   * @param supportLeg: Index of the support leg.
   * @param torsoForces: Forces known acting on base link or torso
   * @param torsoMoments: Moments known acting on base link or torso
   * @param type: Type of joints
   *
   * @return Matrix<Scalar, 2, 1>
   */
  Matrix<Scalar, 2, 1> computeZmpWrtForces(
    const LinkChains& supportLeg,
    Matrix<Scalar, 3, 1>& torsoForces,
    Matrix<Scalar, 3, 1>& torsoMoments,
    const JointStateType& type);

  /**
   * Computes the Zmp of the robot by solving the system dynamics using
   * Newton Euler approach for one chain.
   *
   * @param supportLeg: Index of the support leg.
   * @param chainIndex: The chain index
   * @param torsoForces: Forces known acting on base link or torso
   * @param torsoMoments: Moments known acting on base link or torso
   * @param torques: Torques acting on the chain joints
   * @param type: Type of joints
   *
   * @return Matrix<Scalar, 2, 1>
   */
  Matrix<Scalar, 2, 1> computeZmpWrtChain(
    const LinkChains& supportLeg,
    const LinkChains& chainIndex,
    const JointStateType& type,
    Matrix<Scalar, 3, 1>& torsoForces,
    Matrix<Scalar, 3, 1>& torsoMoments,
    Matrix<Scalar, Dynamic, 1>& torques);

  /**
   * Computes the Zmp of the robot by solving the system dynamics using
   * Newton Euler approach.
   *
   * @param supportLeg: Index of the support leg.
   * @param eeIndex: The end effector index.
   * @param type: Type of joints
   * @param torques: The torque of each joint returned from newton euler recursion
   *
   * @return Matrix<Scalar, 2, 1>
   */
  Matrix<Scalar, 2, 1> computeZmp(
    const LinkChains& supportLeg,
    const JointStateType& type,
    Matrix<Scalar, Dynamic, 1>& torques);

  /**
   * Computes the Zmp of the robot from feet force sensors
   *
   * @param refFrame: Index of the reference foot frame
   *
   * @return Matrix<Scalar, 2, 1>
   */
  Matrix<Scalar, 2, 1> computeFsrZmp(const RobotFeet& refFrame);

  /**
   * Forward Kinematics Solver.
   *
   * @param chainIndex: Index of the limb.
   * @param eeIndex: The end effector index.
   * @param type: Type of joints
   *
   * @return Matrix<Scalar, 4, 4>
   */
  Matrix<Scalar, 4, 4> getForwardEffector(const LinkChains& chainIndex,
    const unsigned& eeIndex,
    const JointStateType& type = JointStateType::actual);

  /**
   * Forward Kinematics Solver.
   *
   * @param chainIndex: Index of the limb.
   * @param endEffector: The end effector transformation matrix from
   *   the final frame of the chain.
   * @param type: Type of joints
   * @return Matrix<Scalar, 4, 4>
   */
  Matrix<Scalar, 4, 4> getForwardEffector(const LinkChains& chainIndex,
    const Matrix<Scalar, 4, 4>& endEffector,
    const JointStateType& type = JointStateType::actual);

#ifdef ALLOW_SYMBOLIC_COMPUTATIONS
  /**
   * Forward kinematics solver for symbolic computations
   * @param chainIndex: Index of the limb
   * @param endEffector: Endeffector matrix in eigen
   * @return A dense matrix of forward kinematics
   */
  SymEngine::DenseMatrix getForwardEffectorSym(
    const unsigned& chainIndex, const Matrix<Scalar, 4, 4> &endEffector);

  /**
   * Forward kinematics solver for symbolic computations
   * @param chainIndex: Index of the limb
   * @param endEffector: Endeffector matrix index
   * @return A dense matrix of forward kinematics
   */
  SymEngine::DenseMatrix getForwardEffectorSym(
    const unsigned& chainIndex, const unsigned& eeIndex);
#endif

  /**
   * @brief getGlobalFootToOtherFoot Returns the transformation
   *   from global base foot to other foot
   * @return
   */
  Matrix<Scalar, 4, 4> getGlobalToOther();

  /**
   * Returns the required joint
   *
   * @param index: Joint Index
   *
   * @return Joint
   */
  boost::shared_ptr<Joint<Scalar> > getJoint(const Joints& index);

  /**
   * Returns the state of the required joint
   *
   * @param index: Joint Index
   * @param type: Type of joints
   *
   * @return Joint
   */
  boost::shared_ptr<JointState<Scalar> > getJointState(
    const Joints& index,
    const JointStateType& type = JointStateType::actual);

  /**
   * Returns the current position of the joint
   *
   * @param index: Joint Index
   * @param type: Type of joints
   *
   * @return Joint position
   */
  Scalar getJointPosition(
    const Joints& index,
    const JointStateType& type = JointStateType::actual);

  /**
   * Returns all the required joints
   *
   * @param startIndex: First joint index.
   * @param nElements: Number of elements including the
   *   first one.
   * @param type: Type of joints
   *
   * @return Vector<boost::shared_ptr<Joint<Scalar> > >
   */
  vector<boost::shared_ptr<Joint<Scalar> > > getJoints(
    const Joints& startIndex = Joints::first,
    const unsigned& nElements = toUType(Joints::count));

  /**
   * Returns all the states of all required joints
   *
   * @param startIndex: First joint index.
   * @param nElements: Number of elements including the
   *   first one.
   * @param type: Type of joints
   *
   * @return Vector<boost::shared_ptr<JointState<Scalar> > >
   */
  vector<boost::shared_ptr<JointState<Scalar> > > getJointStates(
    const Joints& startIndex = Joints::first,
    const unsigned& nElements = toUType(Joints::count),
    const JointStateType& type = JointStateType::actual);

  /**
   * Returns the positions of all required joints
   *
   * @param startIndex: First joint index.
   * @param nElements: Number of elements including the
   *   first one.
   * @param type: Type of joints
   *
   * @return Vector<Scalar, Dynamic, 1>
   */
  Matrix<Scalar, Dynamic, 1> getJointPositions(
    const Joints& startIndex = Joints::first,
    const unsigned& nElements = toUType(Joints::count),
    const JointStateType& type = JointStateType::actual);

  /**
   * Returns the velocities of all required joints
   *
   * @param startIndex: First joint index.
   * @param nElements: Number of elements including the
   *   first one.
   * @param type: Type of joints
   *
   * @return Vector<Scalar, Dynamic, 1>
   */
  Matrix<Scalar, Dynamic, 1> getJointVelocities(
    const Joints& startIndex = Joints::first,
    const unsigned& nElements = toUType(Joints::count),
    const JointStateType& type = JointStateType::actual);

  /**
   * Gets the joint positions for the required chain
   *
   * @param chainIndex: Chain of which the values are required
   * @param type: Type of joints.
   *
   * @return a vector containing the joint values.
   */
  vector<boost::shared_ptr<JointState<Scalar> > > getChainStates(
    const LinkChains& chainIndex,
    const JointStateType& type = JointStateType::actual);

  /**
   * Returns the required link
   *
   * @param index: Index of the link
   *
   * @return LinkInfo
   */
  boost::shared_ptr<LinkInfo<Scalar> >
  getLink(const Links& index);

  /**
   * Returns the required chain
   *
   * @param index: Index of the chain
   *
   * @return LinkChain
   */
  boost::shared_ptr<LinkChain<Scalar> >
  getLinkChain(const LinkChains& index);

  /**
   * @brief getGlobalBaseIndex Returns the global base index
   * @return RobotFeet
   */
  RobotFeet getGlobalBaseIndex();

  /**
   * Returns the current global base chain
   *
   * @return LinkChain
   */
  boost::shared_ptr<LinkChain<Scalar> > getGlobalBase();

  /**
   * Returns the end effectors of the required chain
   *
   * @param chain: Chain index
   * @param index: End effector index
   *
   * @return Matrix<typename Scalar, 4, 4>
   */
  Matrix<Scalar, 4, 4>  getEndEffector(
    const LinkChains& chain, const unsigned& index);

  /**
   * This part of the code has been adapted from Kofinas'
   * NAOKinematics code but redefined using Eigen.
   *
   * Analytical inverse kinematics for left leg of the robot.
   *
   * @param endEffector: The end effector transformation.
   * @param targetT: The desired configuration of the end effector
   *
   * @return vector<Matrix<Scalar, Dynamic, 1>>
   *
   * @author Kofinas Nikos aka eldr4d, 2012 kouretes team
   * @author Vosk
   */
  vector<Matrix<Scalar, Dynamic, 1>> inverseLeftLeg(
    const Matrix<Scalar, 4, 4>& endEffector,
    const Matrix<Scalar, 4, 4>& targetT);

  /**
   * This part of the code has been adapted from Kofinas'
   * NAOKinematics code but redefined using Eigen.
   *
   * Analytical inverse kinematics for right leg of the robot.
   *
   * @param endEffector: The end effector transformation.
   * @param targetpoint: The desired configuration of
   *   the end effector
   *
   * @return vector<Matrix<Scalar, Dynamic, 1>>
   *
   * @author Kofinas Nikos aka eldr4d, 2012 kouretes team
   * @author Vosk
   */
  vector<Matrix<Scalar, Dynamic, 1>> inverseRightLeg(
    const Matrix<Scalar, 4, 4>& endEffector,
    const Matrix<Scalar, 4, 4>& targetT);

  /**
   * This function sets the required end-effector for the left and
   * right legs for now. Update if needed.
   *
   * @param chain: Chain whose end-effector is
   *   to be setup.
   * @param eeIndex: Index of the saved end-effector to
   *   be used.
   * @param ee: The position of the end-effector from the last frame
   *   of the chain.
   */
  void setEndEffector(
    const LinkChains& chain,
    const unsigned& eeIndex,
    const Matrix<Scalar, 4, 1>& ee);

  /**
   * Sets the current global base chain index
   */
  void setGlobalBase(
      const RobotFeet& globalBase, const LegEEs& globalEnd);

  /**
   * This function gets the center of mass with respect to
   * a base frame (Torso, left ankle or right ankle).
   *
   * @param limbIndex: The base limb for com.
   * @param eeIndex: End effector index
   * @param comVector: Output Vector
   * @param type: Type of joints
   */
  void computeComWrtBase(
    const LinkChains& limbIndex,
    const unsigned& eeIndex,
    Matrix<Scalar, 3, 1>& comVector,
    const JointStateType& type = JointStateType::actual);

  /**
   * This function gets the center of mass with respect to
   * a base frame (Torso, left ankle or right ankle) in X-Y plane.
   *
   * @param limbIndex: The base limb for com.
   * @param eeIndex: End effector index
   * @param comVector: Output Vector
   * @param type: Type of joints
   */
  void computeComWrtBase(
    const LinkChains& limbIndex,
    const unsigned& eeIndex,
    Matrix<Scalar, 2, 1>& comVector,
    const JointStateType& type = JointStateType::actual);

  /**
   * This function gets the center of mass with respect to
   * a base frame (Torso, left ankle or right ankle).
   *
   * @param limbIndex: The base limb for com.
   * @param eeIndex: End effector.
   * @param type: Type of joints
   *
   * @return Output center of mass vector
   */
  Matrix<Scalar, 3, 1> computeComWrtBase(
    const LinkChains& limbIndex,
    const unsigned& eeIndex,
    const JointStateType& type = JointStateType::actual);

  bool computeVirtualMass(
    const LinkChains& chainIndex,
    const Matrix<Scalar, 3, 1>& direction,
    const Matrix<Scalar, 4, 4>& endEffector,
    Scalar& virtualMass,
    const JointStateType& type = JointStateType::actual);

  /**
   * Returns the tranformation matrix from torso to center
   *   of the feet.
   *
   * @return Matrix<Scalar, 4, 4>
   */
  Matrix<Scalar, 4, 4> getFeetCenterT();

  /**
   * Returns the transformed vector from foot frame to camera frame
   *
   * @param camIndex: top or bottom cam index
   * @param posInFoot: position vector in foot frame
   *
   * @return Matrix<Scalar, 4, 1>
   */
  Matrix<Scalar, 4, 1> getWorldToCam(
    const CameraId& camIndex, const Matrix<Scalar, 4, 1>& posInFoot);

  /**
   * Returns the current best estimate of com state relative to the given
   * base frame.
   *
   * @param baseFrame: Target reference frame
   * @param eeIndex: Target frame end-effector index
   *
   * @return ComState<Scalar>
   */
  ComState<Scalar> getComStateWrtFrame(
    const LinkChains& baseFrame = -1,
    const unsigned& eeIndex = 0);

  /**
   * Returns the process model used for estimating center of mass
   * state in the given direction
   * @param index: Index that specifies the directions -> 0=x, 1=y
   * @return boost::shared_ptr<ProcessModel<Scalar, 3, 1, 1> >
   */
  boost::shared_ptr<ProcessModel<Scalar, 3, 1, 1> >
  getComModel(const unsigned& index);

  /**
   * Returns the current state of the torso
   *
   * @return boost::shared_ptr<TorsoState<Scalar> >
   */
  boost::shared_ptr<TorsoState<Scalar> > getTorsoState();

  /**
   * Returns the transformation from global base to body center frame
   * @param type joint state type
   * @return Matrix<Scalar, 4, 4>
   */
  Matrix<Scalar, 4, 4> getGlobalToBody(const JointStateType& type = JointStateType::actual);

  /**
   * Returns the transformation from body to global base frame
   * @param type joint state type
   * @return Matrix<Scalar, 4, 4>
   */
  Matrix<Scalar, 4, 4> getBodyToGlobal(const JointStateType& type = JointStateType::actual);

  /**
   * Returns the foot that is on ground. -1 if none.
   *
   * @return int
   */
  RobotFeet getFootOnGround();

  /**
   * Returns the foot spacing distance in y-axis. Useful when robot is standing
   *
   * @return Scalar
   */
  Scalar getFootSpacing();

  /**
   * Returns the motion thread cycle time.
   *
   * @return Scalar
   */
  Scalar getCycleTime();

  unsigned getNJoints()
  {
    return toUType(Joints::count);
  }

  boost::shared_ptr<TaskIkSolver<Scalar> > getTaskSolver() const {
    return tis;
  }

  /**
   * Function to print all the kinematic data for debugging.
   */
  void printKinematicData();

private:
  /**
   * Sets up the robot links and their inertias
   */
  void setupLinksAndInertias();

  /**
   * Sets up the robot joints and their states
   */
  void setupJoints();

  /**
   * Sets up the robot chains
   */
  void setupChains();

  /**
   * Sets up the robot torso and center of mass states
   */
  void setupWBStates();

  /**
   * Sets up the robot head chain
   */
  void setupHeadChain();

  /**
   * Sets up the robot left and right arm chains
   */
  void setupArmChains();

  /**
   * Sets up the robot left and right leg chains
   */
  void setupLegChains();

  /**
   * Sets up the robot chain end effectors
   */
  void setupEndEffectors();

  /**
   * Updates the joint states for the given cycle from memory
   */
  void updateJointStates();

  /**
   * Updates the robot torso state based on sensor readings
   */
  void updateTorsoState();

  /**
   * Updates the robot center of mass state based on sensor readings
   */
  void updateComState();

  /**
   * Updates the transformation of the camera's with respect to the
   * foot.
   */
  void updateFootToCamT();

  /**
   * Updates the transformation of the left and right feet from torso.
   */
  void updateTorsoToFeet();

  /**
   * Updates which foot is on the ground.
   */
  void updateFootOnGround();

  /**
   * This part of the code has been adapted from Kofinas'
   * NAOKinematics code but redefined using Eigen.
   *
   * Prepares the kinematic model by finding the dh matrix with
   * respect to the joint positions.
   *
   * @param ch: the chains to be prepared.
   * @param type: Type of joints
   * @param solveFk: Whether to solve the forward kinematics
   *  along with dh transformations.
   *
   * @author Kofinas Nikos aka eldr4d, 2012 kouretes team
   * @author Vosk
   */
  void prepareDHTransforms(
    const LinkChains& ch = LinkChains::count,
    const JointStateType& type = JointStateType::actual,
    const bool& solveFk = true);

  /**
   * The function calculates the center of mass of the robot with
   * respect to the torso frame of the robot.
   *
   * @param type = Type of joints
   *
   * @return Matrix<Scalar, 3, 1>
   */
  Matrix<Scalar, 3, 1> calculateCenterOfMass(
    const JointStateType& type = JointStateType::actual);

  ///< Base class object.
  MotionModule* motionModule;

  ///< Task ik solver
  boost::shared_ptr<TaskIkSolver<Scalar> > tis;

  ///< Motion cycle time
  Scalar cycleTime;

  ///< Global base reference frame
  RobotFeet globalBase;

  ///< Global end-effector index
  LegEEs globalEnd;

  ///< Transformation from body center torso frame to global base frame
  vector<Matrix<Scalar, 4, 4> > bodyToGlobal;

  ///< Transformation from global base to body center torso frame
  vector<Matrix<Scalar, 4, 4> > globalToBody;

  ///< Rotation matrix from global base to body center torso frame
  vector<Matrix<Scalar, 6, 6> >globalToBodyRotX;

  ///< Torso acceleration of the robot in X-Y-Z
  boost::shared_ptr<TorsoState<Scalar> > torsoState;

  ///< Imu filter
  boost::shared_ptr<ImuFilter<Scalar> > imuFilter;

  ///< Com state of the robot wrt the torso
  boost::shared_ptr<ComState<Scalar> > comState;

  ///< KF-based com state estimator
  vector<boost::shared_ptr<ComEstimator<Scalar, 3> > > comEstimator;

  ///< Total mass of body chains.
  Scalar totalChainsMass;

  ///< Joints vector
  vector<boost::shared_ptr<Joint<Scalar> > > joints;

  ///< Links info vector
  vector<boost::shared_ptr<LinkInfo<Scalar> > > links;

  ///< Link chains vector
  vector<boost::shared_ptr<LinkChain<Scalar> > > linkChains;

  ///< Left foot transformation in torso frame.
  Matrix<Scalar, 4, 4> lFootOnGround;

  ///< Right foot transformation in torso frame.
  Matrix<Scalar, 4, 4> rFootOnGround;

  ///< Upper cam transformation in feet frame.
  Matrix<Scalar, 4, 4> upperCamInFeet;

  ///< Lower cam transformation in feet frame.
  Matrix<Scalar, 4, 4> lowerCamInFeet;

  ///< The variable that defines whether right or left or both feet are
  ///< on the ground
  RobotFeet footOnGround;

  ///< Foot spacing in y-direction
  Scalar footSpacing;

  ///< Torso pitch offset for camera transformation
  Scalar torsoPitchOffset;

  ///< Torso roll offset for camera transformation
  Scalar torsoRollOffset;

  ///< Feet forces s buffer
  boost::circular_buffer<Matrix<Scalar, 2, 1>> feetForcesBuffer;

  ///< Feet forces buffer size
  size_t ffBufferSize;

  /**
   * This part of the code has been adapted from Kofinas'
   * NAOKinematics code.
   *
   * @author Kofinas Nikos aka eldr4d, 2012 kouretes team
   * @author Vosk
   */
  Matrix<Scalar, 4, 4> tBaseLLegInv, tEndLLegInv, rotFixLLeg, rotRLeg;

  ///< Log file path for robot center of mass and zmp estimate
  fstream comLog;

  ///< Json log file for imu data
  JsonLoggerPtr kinDataLogger;
  bool logKinData;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef boost::shared_ptr<KinematicsModule<MType> > KinematicsModulePtr;

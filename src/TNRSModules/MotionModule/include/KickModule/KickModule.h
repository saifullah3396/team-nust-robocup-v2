/**
 * @file MotionModule/include/KickModule/KickModule.h
 *
 * This file declares the class KickModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 15 May 2017
 */

#pragma once

#include "MotionModule/include/MotionBehavior.h"
#include "MotionModule/include/MTypeHeader.h"

enum class JointStateType : unsigned int;
template <typename Scalar>
class BSpline;
template <typename Scalar>
class MaxMomentumEEOpt;
struct MBKickConfig;

/**
 * @class KickModule
 * @brief The base class for defining different kinds of kick engines
 */
template <typename Scalar>
class KickModule : public MotionBehavior<Scalar>
{
public:
  /**
   * @brief KickModule Constructor
   *
   * @param motionModule Pointer to base motion module
   * @param config Configuration of the behavior
   * @param name Name of the behavior
   */
  KickModule(
    MotionModule* motionModule,
    const boost::shared_ptr<MBKickConfig>& config,
    const string& name = "KickModule");

  /**
   * @brief ~KickModule Destructor
   */
  virtual ~KickModule() {}

  /**
   * @brief getType Returns its own child based on the given type
   *
   * @param motionModule: Pointer to base motion module
   * @param cfg: Config of the requested behavior
   *
   * @return boost::shared_ptr<KickModule<Scalar> >
   */
  static boost::shared_ptr<KickModule<Scalar> > getType(
    MotionModule* motionModule, const BehaviorConfigPtr& cfg);

  /**
   * @brief loadExternalConfig See Behavior::loadExternalConfig()
   */
  virtual void loadExternalConfig() override;

protected:
  /**
   * @brief setupPosture Sets the required posture config as a child
   */
  void setupPosture();

  /**
   * @brief setupPosture Sets the required balance config as a child
   */
  virtual void setupBalance();

  /**
   * @brief setTransformFrames Sets the required transformation
   *   frames from support to end effector frame and
   *   torso to support leg frame
   *
   * @return False if transformations are not set correctly
   */
  virtual bool setTransformFrames(const JointStateType& type);

  /**
   * @brief setKickSupportLegs Sets the kicking and support leg frames
   *
   * @return True if successful
   */
  virtual bool setKickSupportLegs();

  /**
   * @brief setEndEffectorXY Sets the end-effector xy-coordinates based on foot contour
   *   approximation
   * @param angle Angle for the normal to contour. Ranges from -90 to
   *   +90 degrees
   *
   * @return True if successful
   */
  bool setEndEffectorXY(const Scalar& angle);

  /**
   * @brief Sets the end-effector Z-X coordinates based on foot contour
   *   approximation
   *
   * @param t Z-X bezier curve parameter [0,...,1] .
   */
  void setEndEffectorZX(const Scalar& t);

  /**
   * @brief setDesBallVel Sets the desired ball velocity based on the required information.
   *   For example, one way to solve for it is using known frictional
   *   parameters. Another could be using a pretrained learning classifier.
   */
  virtual void setDesBallVel();

  /**
   * @brief computeDesImpactVel Finds the desired impact velocity based on
   *   the target distance and solving momentum conservation for foot-ball
   *   collision based on end-effector virtual mass calculations.
   *
   * @param impJoints Joints at the impact condition
   */
  virtual void computeDesImpactVel(const Matrix<Scalar, Dynamic, 1>& impJoints);

  /**
   * @brief Returns true if the kicking leg foot is colliding with support leg foot
   *
   * @return bool
   */
  bool checkFootCollision(
    const Matrix<Scalar, Dynamic, 1>& kickAngles);

  /**
   * @brief logFootContours Log the contours of the feet
   */
  void logFootContours();

  /**
   * @brief plotFootSurfaces Plots the foot front surfaces in 3D
   */
  //void plotFootSurfaces();

  /**
   * @brief makeFootSurfaces3D Makes the feet surface matrix
   */
  //void makeFootSurfaces3D(
  //  const bool& leftFoot, fstream& log, fstream& zxLog, Mat& surfaceMat);

  ///< Ball mass
  static Scalar ballMass;

  ///< Ball radius
  static Scalar ballRadius;

  ///< Ball static friction coefficient
  static Scalar sf;

  ///< Ball rolling friction coefficient
  static Scalar rf;

  ///< Coefficient of damping for ball if damping equation is used
  Scalar coeffDamping;

  ///< Ball Position in base leg frame
  Matrix<Scalar, 3, 1> ballPosition;

  ///< Ball to target direction vector
  Matrix<Scalar, 3, 1> ballToTargetUnit;

  ///< Target Position in base leg frame
  Matrix<Scalar, 3, 1> targetPosition;

  ///< Angle from ball to target
  Scalar targetAngle;

  ///< Distance from ball to target
  Scalar targetDistance;

  ///< Distance between the feet frames.
  Scalar footSpacing;

  ///< Kicking Leg
  LinkChains kickLeg;

  ///< Support Leg
  LinkChains supportLeg;

  ///< End-Effector frame wrt kick leg base
  Matrix<Scalar, 4, 4> endEffector;

  ///< End-effector pose at impact wrt the support leg frame
  Matrix<Scalar, 4, 4> impactPose;

  ///< Transformation matrix from support leg frame to end effector frame
  Matrix<Scalar, 4, 4> supportToKick;

  ///< Transformation matrix from torso to support leg frame
  Matrix<Scalar, 4, 4> torsoToSupport;

  ///< Inverse of torsoToSupport
  Matrix<Scalar, 4, 4> supportToTorso;

  ///< Desired end-effector velocity on impact in cartesian space
  Matrix<Scalar, 3, 1> desImpactVel;

  ///< Desired ball velocity in 1D in target direction
  Scalar desBallVel;

  ///< Whether to compute desImpactVel based on distance // Impulse/Vm based solution
  bool desImpactVelKnown;

  ///< Whether the requested kick is not acheivable
  bool kickFailed = {false};

  ///< Kick task active joints
  vector<bool> kickTaskJoints;

  ///< Arms task active joints
  vector<bool> armsTaskJoints;

  ///< Time taken by the overall kick trajectory
  Scalar totalTimeToKick;

  ///< BSpline defining the left foot contour
  static BSpline<Scalar>* lFootContour;

  ///< BSpline defining the right foot contour
  static BSpline<Scalar>* rFootContour;

  ///< Foot rectangle defined by position vectors to four corners
  static vector<Matrix<Scalar, 3, 1> > footRect;

  ///< Maximum momentum optimizer is friend as it uses kick module
  ///< functions and variables
  friend class MaxMomentumEEOpt<Scalar>;

  ///< Foot contour matrix in ZX direction
  static Matrix<Scalar, 4, 4> bezierMat;
  static Matrix<Scalar, 4, 3> contourMat;

private:
  /**
   * Returns the cast of config to MBKickConfigPtr
   */
  boost::shared_ptr<MBKickConfig> getBehaviorCast();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef boost::shared_ptr<KickModule<MType> > KickModulePtr;

/**
 * @file MotionModule/BalanceModule/Types/ZmpControl.h
 *
 * This file declares the class ZmpControl
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018
 */

#pragma once

#include <fstream>
#include "MotionModule/include/BalanceModule/BalanceModule.h"

struct ZmpControlConfig;
template <typename Scalar>
class MotionTask;
typedef boost::shared_ptr<MotionTask<MType> > TaskPtr;
template <typename Scalar>
struct ZmpRef;
template <typename Scalar>
class ZmpRefGenerator;
template <typename Scalar>
class ZmpPreviewController;

/**
 * @class ZmpControl
 * @brief The class for controlling the center of mass movement through
 *   a PID controller
 */
template <typename Scalar>
class ZmpControl : public BalanceModule<Scalar>
{
public:
  /**
   * @brief ZmpControl Constructor
   * @param motionModule Pointer to base motion module
   * @param config Configuration of this behavior
   */
  ZmpControl(
    MotionModule* motionModule,
    const boost::shared_ptr<ZmpControlConfig>& config);
  
  /**
   * @brief ~ZmpControl Destructor
   */
  ~ZmpControl() final;

  /**
   * @brief initiate See Behavior::initiate()
   */
  bool initiate() final;

  /**
   * @brief reinitiate See Behavior::reinitiate()
   */
  void reinitiate(const BehaviorConfigPtr& cfg) final;

  /**
   * @brief update See Behavior::update()
   */
  void update() final;

  /**
   * @brief finish See Behavior::finish()
   */
  void finish() final;

  /**
   * @brief loadExternalConfig See Behavior::loadExternalConfig()
   */
  void loadExternalConfig() final;
private:
  /**
   * @brief trackZmp Tracks the desired reference of ZMP by following
   *   using a preview controller.
   */
  void trackZmp();

  /**
   * @brief getBehaviorCast Returns the cast of config to ZmpControlConfigPtr
	 */
  boost::shared_ptr<ZmpControlConfig> getBehaviorCast();
  
  //! Preview controllers for x-y directions
  vector<ZmpPreviewController<Scalar>*> controllers;
  
  //! Zmp reference generator for x-y directions
  boost::shared_ptr<ZmpRefGenerator<Scalar>> refGenerator;

  //! Desired zmp references relative to support leg in x-y directions
  boost::shared_ptr<ZmpRef<Scalar>> zmpRef;
  
  //! Tasks vector for solving whole-body inverse kinematics
  vector<TaskPtr> tasks;

  //! Desired com position in each update
  Matrix<Scalar, 3, 1> desComPosition;
  
  //! Other leg (from support) index
  LinkChains otherLeg;

  //! Log for storing center of mass data
  fstream comLog;

  //! Log for storing zmp ref data
  fstream zmpRegLog;

  //! Posture task target
  Matrix<Scalar, Dynamic, 1> postureTarget;

  //! Number of zmp reference previews required
  static unsigned nPreviews;

  //! Task weights and gains
  static vector<Scalar> taskWeights;
  static vector<Scalar> taskGains;

  enum IkTasks : unsigned int {
    com,
    posture,
    contact,
    torso,
    count
  };
};

typedef boost::shared_ptr<ZmpControl<MType> > ZmpControlPtr;

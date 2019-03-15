/**
 * @file MotionModule/include/HeadControl/HeadControl.h
 *
 * This file declares the class HeadControl
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 20 Nov 2017 
 */

#pragma once

#include "MotionModule/include/MotionBehavior.h"

enum class HeadTargetTypes : unsigned int;
struct MBHeadControlConfig;

/** 
 * @class HeadControl
 * @brief The class for controlling the robot head movement
 */
template <typename Scalar>
class HeadControl : public MotionBehavior<Scalar>
{
public:
  /**
   * @brief HeadControl Constructor
   *
   * @param motionModule Pointer to base motion module
   * @param config Configuration of the behavior
   * @param name Name of the behavior
   */
  HeadControl(
    MotionModule* motionModule,
    const boost::shared_ptr<MBHeadControlConfig>& config,
    const string& name = "HeadControl");

  /**
   * @brief ~HeadControl Destructor
   */
  virtual ~HeadControl() {}

  /**
   * @brief getType Returns its own child based on the given type
   *
   * @param motionModule: Pointer to base motion module
   * @param cfg: Config of the requested behavior
   *
   * @return BehaviorConfigPtr
   */
  static boost::shared_ptr<HeadControl<Scalar> > getType(
    MotionModule* motionModule, const BehaviorConfigPtr& cfg);

  /**
   * @brief loadExternalConfig See Behavior::loadExternalConfig
   */
  virtual void loadExternalConfig() override {}
  
protected:
  /**
   * Returns true if a target of type targetType is found and saves its
   * x-y-z coordinates in the input variables
   * 
   * @param targetType: Target type
   * @param targetXY: x-y coordinate output if the target is found
   * @param targetZ: z coordinate output if the target is found
   */ 
  bool findTarget(
    const HeadTargetTypes& targetType, 
    cv::Point_<Scalar>& targetXY,
    Scalar& targetZ);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

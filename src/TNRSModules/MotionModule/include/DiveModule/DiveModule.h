/**
 * @file MotionModule/include/DiveModule/DiveModule.h
 *
 * This file declares the class DiveModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 26 Dec 2017 
 */

#pragma once

#include "MotionModule/include/MotionBehavior.h"
#include "MotionModule/include/DiveModule/KeyFrameDiveTypes.h"

struct MBDiveConfig;

/** 
 * @class DiveModule
 * @brief The class for generating different types of dives on the 
 *   robot.
 */
template <typename Scalar>
class DiveModule : public MotionBehavior<Scalar>
{
public:
  /**
   * @brief DiveModule Constructor
   *
   * @param motionModule Pointer to base motion module
   * @param config Configuration of the behavior
   * @param name Name of the behavior
   */
  DiveModule(
    MotionModule* motionModule,
    const boost::shared_ptr<MBDiveConfig>& config,
    const string& name = "DiveModule");

  /**
   * @brief ~DiveModule Destructor
   */
  virtual ~DiveModule() {}

  /**
   * @brief getType Returns its own child based on the given type
   *
   * @param motionModule: Pointer to base motion module
   * @param cfg: Config of the requested behavior
   *
   * @return BehaviorConfigPtr
   */
  static boost::shared_ptr<DiveModule<Scalar> > getType(
    MotionModule* motionModule, const BehaviorConfigPtr& cfg);

  /**
   * @brief loadExternalConfig See Behavior::loadExternalConfig
   */
  virtual void loadExternalConfig() override {}

protected:
  ///< The final posture state after dive
  PostureState endPosture;

  ///< Motion execution time updated after each update
  Scalar execTime;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef boost::shared_ptr<DiveModule<MType> > DiveModulePtr;

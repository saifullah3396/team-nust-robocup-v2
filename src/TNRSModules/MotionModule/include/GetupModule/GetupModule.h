/**
 * @file MotionModule/include/GetupModule/GetupModule.h
 *
 * This file declares the class GetupModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 26 Dec 2017
 */

#pragma once

#include "MotionModule/include/GetupModule/KeyFrameGetupTypes.h"
#include "MotionModule/include/MotionBehavior.h"

struct MBGetupModuleConfig;

/**
 * @class GetupModule
 * @brief The class for generating motions for standing up if a robot
 *   is fallen.
 */
template<typename Scalar>
class GetupModule : public MotionBehavior<Scalar>
{
public:
  /**
   * @brief GetupModule Constructor
   *
   * @param motionModule Pointer to base motion module
   * @param config Configuration of the behavior
   * @param name Name of the behavior
   */
  GetupModule(
    MotionModule* motionModule,
    const boost::shared_ptr<MBGetupConfig>& config,
    const string& name = "GetupModule");

  /**
   * @brief ~GetupModule Destructor
   */
  virtual ~GetupModule() {}

  /**
   * @brief getType Returns its own child based on the given type
   *
   * @param motionModule: Pointer to base motion module
   * @param cfg: Config of the requested behavior
   *
   * @return BehaviorConfigPtr
   */
  static boost::shared_ptr<GetupModule<Scalar> > getType(
    MotionModule* motionModule, const BehaviorConfigPtr& cfg);

  /**
   * @brief loadExternalConfig See Behavior::loadExternalConfig
   */
  virtual void loadExternalConfig() override {}
  
protected:
  //! The final posture state after getting up
  PostureState endPosture;
  
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef boost::shared_ptr<GetupModule<MType> > GetupModulePtr;

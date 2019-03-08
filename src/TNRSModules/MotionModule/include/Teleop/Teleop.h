/**
 * @file MotionModule/include/Teleop/Teleop.h
 *
 * This file declares the class Teleop
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 26 Dec 2017 
 */

#pragma once

#include "MotionModule/include/MotionBehavior.h"
#include "MotionModule/include/MTypeHeader.h"

struct MBTeleopConfig;

/** 
 * @class Teleop
 * @brief The class for generating different types of dives on the 
 *   robot.
 */
template <typename Scalar>
class Teleop : public MotionBehavior<Scalar>
{
public:
  /**
   * @brief Teleop Constructor
   *
   * @param motionModule Pointer to base motion module
   * @param config Configuration of the behavior
   * @param name Name of the behavior
   */
  Teleop(
    MotionModule* motionModule,
    const boost::shared_ptr<MBTeleopConfig>& config,
    const string& name = "Teleop");

  /**
   * @brief ~Teleop Destructor
   */
  virtual ~Teleop() {}
  
  /**
   * @brief getType Returns its own child based on the given type
   * 
   * @param motionModule: Pointer to base motion module
   * @param cfg: Config of the requested behavior
   * 
   * @return boost::shared_ptr<Teleop<Scalar> >
   */
  static boost::shared_ptr<Teleop<Scalar> > getType(
    MotionModule* motionModule, const BehaviorConfigPtr& cfg);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef boost::shared_ptr<Teleop<MType> > TeleopPtr;

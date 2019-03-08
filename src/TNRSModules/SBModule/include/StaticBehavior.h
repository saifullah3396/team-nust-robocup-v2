/**
 * @file SBModule/include/StaticBehavior.h
 *
 * This file declares the class StaticBehavior
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#pragma once

#include <alproxies/almotionproxy.h>
#include "TNRSBase/include/MemoryBase.h"
#include "BehaviorManager/include/Behavior.h"

class SBModule;
#ifdef NAOQI_MOTION_PROXY_AVAILABLE
//! Naoqi motion proxy object
typedef boost::shared_ptr<AL::ALMotionProxy> ALMotionProxyPtr;
#endif

/**
 * @class StaticBehavior
 * @brief A base class for all kinds of static behaviors
 */
class StaticBehavior : public Behavior, public MemoryBase
{
public:
  /**
   * @brief StaticBehavior Constructor
   *
   * @param sbModule Pointer to base static behaviors module
   * @param config Configuration of the behavior
   * @param name Name of the behavior
   */
  StaticBehavior(
    SBModule* sbModule,
    const BehaviorConfigPtr& config,
    const string& name = "StaticBehavior");

  /**
   * @brief ~StaticBehavior Destructor
   */
  virtual ~StaticBehavior() {}

protected:
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  /**
   * @brief naoqiStiffnessInterpolation Interpolates stiffness
   *   using naoqi stiffness interpolation
   *
   * @param ids Ids of all the joints under consideration
   * @param timeLists Times for interpolation of each joint
   * @param stiffnessLists Stiffnesses for interpolation of each joint
   * @param postCommand Whether this command is to be run in a
   *   separate thread or as a blocking call
   */
  void naoqiStiffnessInterpolation(
    const vector<unsigned>& ids,
    const AL::ALValue& timeLists,
    const AL::ALValue& stiffnessLists,
    const bool& postCommand);
  #endif

  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  ALMotionProxyPtr motionProxy; //! Naoqi motion proxy object
  #endif
  SBModule* sbModule; //! Pointer to base static module
};

/**
 * @file GBModule/include/GeneralBehavior.h
 *
 * This file declares the class GeneralBehavior
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#pragma once

#ifndef V6_CROSS_BUILD
#include <alproxies/almotionproxy.h>
#else
#include <qi/anyobject.hpp>
#endif
#include "TNRSBase/include/MemoryBase.h"
#include "BehaviorManager/include/Behavior.h"

#ifndef V6_CROSS_BUILD
  #define NAOQI_MOTION_PROXY_TYPE ALMotionProxyPtr
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  ///< Naoqi motion proxy object
  typedef boost::shared_ptr<AL::ALMotionProxy> ALMotionProxyPtr;
  #endif
#else
  #define NAOQI_MOTION_PROXY_TYPE qi::AnyObject
#endif

class GBModule;

/**
 * @class GeneralBehavior
 * @brief A base class for all kinds of static behaviors
 */
class GeneralBehavior : public Behavior, public MemoryBase
{
public:
  /**
   * @brief GeneralBehavior Constructor
   *
   * @param gbModule Pointer to base static behaviors module
   * @param config Configuration of the behavior
   * @param name Name of the behavior
   */
  GeneralBehavior(
    GBModule* gbModule,
    const BehaviorConfigPtr& config,
    const string& name = "GeneralBehavior");

  /**
   * @brief ~GeneralBehavior Destructor
   */
  virtual ~GeneralBehavior() {}

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
  #ifndef V6_CROSS_BUILD
  void naoqiStiffnessInterpolation(
    const vector<unsigned>& ids,
    const AL::ALValue& timeLists,
    const AL::ALValue& stiffnessLists,
    const bool& postCommand);
  #else
  void naoqiStiffnessInterpolation(
    const vector<unsigned>& ids,
    const vector<vector<float> >& timeLists,
    const vector<vector<float> >& stiffnessLists,
    const bool& postCommand);
  #endif
  #endif

  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  NAOQI_MOTION_PROXY_TYPE motionProxy; ///< Naoqi motion proxy object
  #endif
  GBModule* gbModule; ///< Pointer to base static module
};

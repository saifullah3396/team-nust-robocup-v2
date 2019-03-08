/**
 * @file SBModule/include/WhistleDetector/WhistleDetector.h
 *
 * This file declares the class WhistleDetector
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#pragma once

#include "SBModule/include/StaticBehavior.h"

struct SBWDConfig;

/**
 * @class WhistleDetector
 * @brief Base class for behaviors to detect the whistle
 */ 
class WhistleDetector : public StaticBehavior
{
public:
  /**
   * @brief WhistleDetector Constructor
   * 
   * @param sbModule: Pointer to base static behaviors module
   * @param config: Configuration of the behavior
   * @param name: Name of the behavior
   */
  WhistleDetector(
	SBModule* sbModule,
  const boost::shared_ptr<SBWDConfig>& config,
  const string& name = "WhistleDetector");

  /**
   * @brief ~WhistleDetector Destructor
   */
  virtual ~WhistleDetector() {}
  
  /**
   * @brief getType Returns its own child based on the given type
   * 
   * @param sbModule Pointer to base static behaviors module
   * @param cfg Config of the requested behavior
   * 
   * @return boost::shared_ptr<WhistleDetector>
   */
  static boost::shared_ptr<WhistleDetector> getType(
    SBModule* sbModule, const BehaviorConfigPtr& type);

protected:
 /**
  * @brief getBehaviorCast Returns the casts of config to SBWDConfigPtr
  */ 
  boost::shared_ptr<SBWDConfig> getBehaviorCast();

  /**
   * @brief whistleAction Action to perform when the whistle is detected
   */
  void whistleAction();

  //! Time as in how long should we check for whistle
  float timeToDetect;
  
  //! Behavior time after initiation
  float execTime;
};

typedef boost::shared_ptr<WhistleDetector> WhistleDetectorPtr;

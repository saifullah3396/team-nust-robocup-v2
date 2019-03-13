/**
 * @file GBModule/include/WhistleDetector/WhistleDetector.h
 *
 * This file declares the class WhistleDetector
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#pragma once

#include "GBModule/include/GeneralBehavior.h"

struct GBWDConfig;

/**
 * @class WhistleDetector
 * @brief Base class for behaviors to detect the whistle
 */ 
class WhistleDetector : public GeneralBehavior
{
public:
  /**
   * @brief WhistleDetector Constructor
   * 
   * @param gbModule: Pointer to base static behaviors module
   * @param config: Configuration of the behavior
   * @param name: Name of the behavior
   */
  WhistleDetector(
	GBModule* gbModule,
  const boost::shared_ptr<GBWDConfig>& config,
  const string& name = "WhistleDetector");

  /**
   * @brief ~WhistleDetector Destructor
   */
  virtual ~WhistleDetector() {}
  
  /**
   * @brief getType Returns its own child based on the given type
   * 
   * @param gbModule Pointer to base static behaviors module
   * @param cfg Config of the requested behavior
   * 
   * @return boost::shared_ptr<WhistleDetector>
   */
  static boost::shared_ptr<WhistleDetector> getType(
    GBModule* gbModule, const BehaviorConfigPtr& type);

protected:
 /**
  * @brief getBehaviorCast Returns the casts of config to GBWDConfigPtr
  */ 
  boost::shared_ptr<GBWDConfig> getBehaviorCast();

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

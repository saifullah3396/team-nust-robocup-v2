/**
 * @file GBModule/include/GBManager.h
 *
 * This file declares the class GBManager.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 02 Aug 2018
 */

#pragma once

#include "BehaviorManager/include/BehaviorManager.h"

//! Forward declaration
class GBModule;

/**
 * @class GBManager 
 * @brief Handles the initiation and execution of static behaviors
 */
class GBManager : public BehaviorManager
{
public:
  /**
   * @brief GBManager Constructor
   * 
   * @param gbModule: Pointer to the base static behaviors module
   */
  GBManager(GBModule* gbModule);

  /**
   * @brief ~GBManager Destructor
   */
  ~GBManager() {}
private:
  /**
   * @brief makeBehavior See BehaviorManager::makeBehavior()
   */
  bool makeBehavior(BehaviorPtr& behavior, const BehaviorConfigPtr& cfg) final;
  
  //! Pointer to the base static behaviors module
  GBModule* gbModule;
};
typedef boost::shared_ptr<GBManager> GBManagerPtr;

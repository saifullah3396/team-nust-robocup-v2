/**
 * @file SBModule/include/SBManager.h
 *
 * This file declares the class SBManager.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 02 Aug 2018
 */

#pragma once

#include "BehaviorManager/include/BehaviorManager.h"

//! Forward declaration
class SBModule;

/**
 * @class SBManager 
 * @brief Handles the initiation and execution of static behaviors
 */
class SBManager : public BehaviorManager
{
public:
  /**
   * @brief SBManager Constructor
   * 
   * @param sbModule: Pointer to the base static behaviors module
   */
  SBManager(SBModule* sbModule);

  /**
   * @brief ~SBManager Destructor
   */
  ~SBManager() {}
private:
  /**
   * @brief makeBehavior See BehaviorManager::makeBehavior()
   */
  bool makeBehavior(BehaviorPtr& behavior, const BehaviorConfigPtr& cfg) final;
  
  //! Pointer to the base static behaviors module
  SBModule* sbModule;
};
typedef boost::shared_ptr<SBManager> SBManagerPtr;

/**
 * @file PlanningModule/ExternalInterface/Types/UserRequestsHandler.h
 *
 * This file declares the class UserRequestsHandler
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017
 */

#pragma once

#include "PlanningModule/include/PlanningBehaviors/ExternalInterface/ExternalInterface.h"
#include "TNRSBase/include/DebugBase.h"

/**
 * @class UserRequestsHandler
 * @brief The class for defining a interface to interact with the NIHA
 *   cognition module
 */
class UserRequestsHandler : public ExternalInterface, public DebugBase
{
  INIT_DEBUG_BASE_(
    //! Option to send total module time
    (vector<float>, jointCommands, vector<float>(toUType(Joints::count), NAN)),
  )
public:
  /**
   * @brief UserRequestsHandler Constructor
   * 
   * @param planningModule: pointer to parent planning module
   * @param config: Configuration of this behavior
   */
  UserRequestsHandler(
    PlanningModule* planningModule, 
    const BehaviorConfigPtr& config) :
    ExternalInterface(planningModule, config, "UserRequestsHandler"),
    DebugBase("UserRequestsHandler", this)
  {
  }

  /**
   * @brief UserRequestsHandler Destructor
   */
  ~UserRequestsHandler() final {}

  /**
   * Derived from Behavior
   */ 
  bool initiate() final;
  void update() final;
  void finish() final;
  
private:
  /**
   * * Returns the config casted as UserRequestsHandlerConfigPtr
   */ 
  UserRequestsHandlerConfigPtr getBehaviorCast();
};

typedef boost::shared_ptr<UserRequestsHandler> UserRequestsHandlerPtr;

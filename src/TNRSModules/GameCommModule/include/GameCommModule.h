/**
 * @file GameCommModule/GameCommModule.h
 *
 * This file declares the class GameCommModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#pragma once

#include <type_traits>
#include "TNRSBase/include/DebugBase.h"
#include "TNRSBase/include/BaseIncludes.h"

class TeamComm;
typedef boost::shared_ptr<TeamComm> TeamCommPtr;

/**
 * @class GameCommModule
 * @brief This class defines all the functions and algorithms for data
 *   that are necessary for communication between the robots and the game
 *   controller
 */
class GameCommModule : public BaseModule
{
public:
  DECLARE_INPUT_CONNECTOR(
    gameCommThreadPeriod,
    playerNumber,
    teamNumber,
    teamPort,
    robotFallen,
    robotPose2D,
    moveTarget,
    kickTarget,
    ballInfo,
    robotIntention,
    robotLocalized,
    positionConfidence,
    sideConfidence
  )
  DECLARE_OUTPUT_CONNECTOR(
    teamRobots
  )

  /**
   * @brief GameCommModule Constructor
   * @param teamNUSTSPL pointer to base class which is in this case
   *   the TeamNUSTSPL class
   */
  GameCommModule(void* teamNUSTSPL);

  /**
   * @brief ~GameCommModule Destructor
   */
  ~GameCommModule()
  {
    delete inputConnector;
    delete outputConnector;
  }

  /**
   * See BaseModule::init()
   */
  void init() final;

  /**
   * See BaseModule::mainRoutine()
   */
  void mainRoutine() final;
  
  /**
   * See BaseModule::handleRequests()
   */
  void handleRequests() final;

  /**
   * See BaseModule::initMemoryConn()
   */
  void initMemoryConn() final;

  /**
   * See BaseModule::setThreadPeriod()
   */
  void setThreadPeriod() final;

private:
  //! Pointer to TeamComm class
  TeamCommPtr teamComm;
};

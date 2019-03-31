/**
 * @file GameCommModule/include/GameCommRequest.h
 *
 * This file defines the class GameCommRequest
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#pragma once

#include <opencv2/core/core.hpp>
#include "TeamNUSTSPL/include/TNSPLModuleIds.h"
#include "TNRSBase/include/ModuleRequest.h"
#include "Utils/include/EnumUtils.h"

using namespace cv;

/**
 * Types of request valid for GameCommModule
 *
 * @enum GameCommRequestIds
 */
enum class GameCommRequestIds {
};

/**
 * @class GameCommRequest
 * @brief A module request that can be handled by GameCommModule
 */
class GameCommRequest : public ModuleRequest
{
public:
  /**
   * Constructor
   *
   * @param id: Id of the control request
   */
  GameCommRequest(const GameCommRequestIds& id) :
    ModuleRequest(TNSPLModules::gameComm, toUType(id))
  {
  }
};
typedef boost::shared_ptr<GameCommRequest> GameCommRequestPtr;

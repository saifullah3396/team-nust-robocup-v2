/**
 * @file Utils/include/DataHolders/CommMsgTypes.h
 *
 * This file declares the enumeration CommMsgTypes
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#pragma once

#include "Utils/include/EnumUtils.h"

/**
 * Enumeration for all the communication msg types we can send to the
 * debugger
 *
 * @enum CommMsgTypes
 */
enum class CommMsgTypes : unsigned int
{
  memory = 0,
  logText,
  //topImage,
  //bottomImage,
  footsteps,
  pfStates,
  occupancyGrid,
  knownLandmarks,
  unknownLandmarks,
  nihaCognitionHeader,
  nihaCognitionData,
  userCmd,
  first = memory,
  last = userCmd
};
DECLARE_SPECIALIZED_ENUM(CommMsgTypes)

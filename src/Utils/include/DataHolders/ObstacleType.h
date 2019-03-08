/**
 * @file Utils/include/DataHolders/ObstacleType.h
 *
 * This file defines the enumeration ObstacleType
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 6 Jan 2018
 */

#pragma once

/**
 * Enumeration for the possible type of obstacles
 *
 * @enum ObstacleType
 */
enum class ObstacleType
: unsigned int
{
  unknown = 0,
  goalPost,
  opponent,
  teammate,
  opponentFallen,
  teammateFallen,
  count
};

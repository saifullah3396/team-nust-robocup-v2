/**
 * @file Utils/include/DataHolders/PostureState.h
 *
 * This file declares the enumeration PostureState
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#pragma once

/**
 * Enumeration for all the predefined robot postures.
 *
 * @enum Posture
 */
enum class PostureState
: unsigned int {
  crouch = 0,
  sit,
  standZero,
  stand,
  standHandsBehind,
  standWalk,
  standKick,
  staticPostures,
  getupRead = staticPostures,
  fallFront,
  fallBack,
  fallingFront,
  fallingBack,
  fallSit,
  diveInPlace,
  diveLeft,
  diveRight,
  diveSumo,
  unknown
};

/**
 * @file MotionModule/include/MotionBehaviorIds.h
 *
 * This file declares the enumeration for all motion behaviors and their
 * child types
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 03 Feb 2017
 */

#pragma once

/**
 * Enumeration for all motion behaviors
 *
 * @enum MBIds
 */
enum class MBIds
: unsigned int {
  posture,
  kick,
  movement,
  ballThrow,
  balance,
  headControl,
  dive,
  getup,
  motionPlayback,
  teleop,
  count
};

/**
 * Enumeration for all possible posture behavior types
 *
 * @enum MBPostureTypes
 */
enum class MBPostureTypes
: unsigned int {
  interpToPosture,
  count
};

/**
 * Enumeration for all possible kick behavior types
 *
 * @enum MBKickTypes
 */
enum class MBKickTypes
: unsigned int {
  jsoImpKick,
  jse2DImpKick,
  cSpaceBSplineKick,
  count
};

/**
 * Enumeration for all possible ball throw behavior types
 *
 * @enum MBBallThrowTypes
 */
enum class MBBallThrowTypes
: unsigned int {
  wbBallThrow,
  count
};

/**
 * Enumeration for all possible balance behavior types
 *
 * @enum MBBalanceTypes
 */
enum class MBBalanceTypes
: unsigned int {
  mpComControl,
  pidComControl,
  zmpControl,
  count
};

/**
 * Enumeration for all possible movement behavior types
 *
 * @enum MBMovementTypes
 */
enum class MBMovementTypes
: unsigned int {
  naoqiFootsteps,
  naoqiMoveToward,
  speedWalk,
  kinResolutionWalk,
  count
};

/**
 * Enumeration for all possible head control behavior types
 *
 * @enum MBHeadControlTypes
 */
enum class MBHeadControlTypes
: unsigned int {
  headScan,
  headTargetTrack,
  count
};

/**
 * Enumeration for all possible dive behavior types
 *
 * @enum MBDiveTypes
 */
enum class MBDiveTypes
: unsigned int {
  kfmDive,
  handSaveDive,
  count
};

/**
 * Enumeration for all possible getting up behavior types
 *
 * @enum MBGetupTypes
 */
enum class MBGetupTypes
: unsigned int {
  kfmGetup,
  count
};

/**
 * Enumeration for all possible motion playback behavior types
 *
 * @enum MBMotionPlaybackTypes
 */
enum class MBMotionPlaybackTypes
: unsigned int {
  replayStoredMB,
  count
};

/**
 * Enumeration for all possible teleoperation behavior types
 *
 * @enum MBTeleopTypes
 */
enum class MBTeleopTypes
: unsigned int {
  teleopJoints,
  count
};

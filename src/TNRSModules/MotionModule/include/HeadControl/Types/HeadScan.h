/**
 * @file MotionModule/HeadControl/Types/HeadScan.h
 *
 * This file declares the class HeadScan
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#pragma once

#include "BehaviorManager/include/StateMachineMacros.h"
#include "MotionModule/include/HeadControl/HeadControl.h"

struct HeadScanConfig;

template <typename Scalar>
class HeadScan : public HeadControl<Scalar>
{
public:
  /**
   * @brief HeadScan Constructor
   *
   * @param motionModule Pointer to base motion module
   * @param config Configuration of this behavior
   */
  HeadScan(
    MotionModule* motionModule,
    const boost::shared_ptr<HeadScanConfig>& config);

  /**
   * @brief ~HeadScan Destructor
   */
  ~HeadScan() final {}

  /**
   * @brief initiate See Behavior::initiate()
   */
  bool initiate() final;

  /**
   * @brief update See Behavior::update()
   */
  void update() final;

  /**
   * @brief finish See Behavior::finish()
   */
  void finish() final;
private:
  /**
   * Returns the cast of config to HeadScanConfigPtr
   */
  /**
   * @brief getBehaviorCast Returns the casted config
   * @return boost::shared_ptr<HeadScanConfig>
   */
  boost::shared_ptr<HeadScanConfig> getBehaviorCast();

  /**
   * @brief setScanTarget Sets the current scan target
   * @param yaw Target head yaw
   * @param pitch Target head pitch
   */
  void setScanTarget(
    const Scalar& yaw, const Scalar& pitch);

  /**
   * @brief waitOnTargetReached Waits for scan on reaching the target
   * @return True if it should still wait
   */
  bool waitOnTargetReached();

  /**
   * @brief scanEnv Scans the environment
   */
  void scanEnv();

  //! Finite state machine for this behavior
  DECLARE_FSM(fsm, HeadScan<Scalar>)

  //! MidScan: Look in the middle
  DECLARE_FSM_STATE(HeadScan<Scalar>, MidScan, midScan, onStart, onRun,)

  //! LeftScan: Scan left direction
  DECLARE_FSM_STATE(HeadScan<Scalar>, LeftScan, leftScan, onStart, onRun,)

  //! RightScan: Scan right direction
  DECLARE_FSM_STATE(HeadScan<Scalar>, RightScan, rightScan, onStart, onRun,)
};

typedef boost::shared_ptr<HeadScan<MType> > HeadScanPtr;

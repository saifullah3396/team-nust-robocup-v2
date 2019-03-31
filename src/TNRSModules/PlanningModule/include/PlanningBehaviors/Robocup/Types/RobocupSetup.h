/**
 * @file PlanningModule/PlanningBehaviors/RobocupSetup.h
 *
 * This file declares the class RobocupSetup.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 27 Jul 2018
 */

#pragma once

#include "PlanningModule/include/PlanningBehaviors/Robocup/Robocup.h"

struct RobocupSetupConfig;

/**
 * @class RobocupSetup
 * @brief The class for defining the robocup startup sequence
 */
class RobocupSetup : public Robocup
{
public:
  /**
   * Constructor
   *
   * @param planningModule: pointer to parent planning module
   * @param config: Configuration of this behavior
   */
  RobocupSetup(
    PlanningModule* planningModule,
    const boost::shared_ptr<RobocupSetupConfig>& config);
  /**
   * Destructor
   */
  ~RobocupSetup()
  {
  }

  /**
   * Derived from Behavior
   */
  bool initiate() final;
  void update() final;
  void finish() final;
  virtual void loadExternalConfig() final {}

private:
  /**
   * Returns the config casted as RobocupSetupConfigPtr
   */
  boost::shared_ptr<RobocupSetupConfig> getBehaviorCast();

  void startupAction();
  void cfgHandlingAction();
  void readySequenceAction();
  void getInPositionAction();
  void goingToPositionAction();
  void setSequenceAction();
  void gameplaySequenceAction();

  bool setSequenceFinished;
  unsigned behaviorState;
  enum BehaviorState {
    startup,
    robocupCfg,
    readySequence,
    getInPosition,
    goingToPosition,
    setSequence,
    gameplaySequence
  };
};

typedef boost::shared_ptr<RobocupSetup> RobocupSetupPtr;

/**
 * @file PlanningBehaviors/TestSuite/Types/NavigationTestSuite.h
 *
 * This file declares the class NavigationTestSuite
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#pragma once

#include "BehaviorManager/include/StateMachineMacros.h"
#include "PlanningModule/include/PlanningBehaviors/TestSuite/TestSuite.h"

struct NavigationTestSuiteConfig;

/**
 * @class NavigationTestSuite
 * @brief The class for testing localization module functionality
 */
class NavigationTestSuite : public TestSuite
{
public:
  /**
   * @brief NavigationTestSuite Constructor
   * @param planningModule Pointer to base planning module
   * @param config Configuration of this behavior
   */
  NavigationTestSuite(
    PlanningModule* planningModule,
    const boost::shared_ptr<NavigationTestSuiteConfig>& config);

  /**
   * @brief ~NavigationTestSuite Destructor
   */
  ~NavigationTestSuite() final {}

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

  /**
   * @brief loadExternalConfig See Behavior::loadExternalConfig()
   */
  void loadExternalConfig() final {}

protected:
  /**
   * @brief getBehaviorCast Returns the config casted as NavigationTestSuiteConfigPtr
   */
  boost::shared_ptr<NavigationTestSuiteConfig> getBehaviorCast();

  //! Finite state machine for this behavior
  DECLARE_FSM(fsm, NavigationTestSuite)

  //! GoToTarget: Tests sending the robot to a specified target
  DECLARE_FSM_STATE(NavigationTestSuite, GoToTarget, goToTarget, onStart, onRun,)

  //! GoalChangedReplan: Tests replanning of path on goal change
  DECLARE_FSM_STATE(NavigationTestSuite, GoalChangedReplan, goalChangedReplan, onStart, onRun,)

  //! InvalidPathReplan: Tests replanning of path on invalidity of path
  DECLARE_FSM_STATE(NavigationTestSuite, InvalidPathReplan, invalidPathReplan, onStart, onRun,)

  enum MBManagerIds {
    MOTION_1
  };
};

typedef boost::shared_ptr<NavigationTestSuite> NavigationTestSuitePtr;

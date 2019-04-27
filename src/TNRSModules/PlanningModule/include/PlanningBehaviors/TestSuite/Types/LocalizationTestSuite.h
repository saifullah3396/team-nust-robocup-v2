/**
 * @file PlanningBehaviors/TestSuite/Types/LocalizationTestSuite.h
 *
 * This file declares the class LocalizationTestSuite
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#pragma once

#include "BehaviorManager/include/StateMachineMacros.h"
#include "PlanningModule/include/PlanningBehaviors/TestSuite/TestSuite.h"

struct LocalizationTestSuiteConfig;

/**
 * @class LocalizationTestSuite
 * @brief The class for testing localization module functionality
 */
class LocalizationTestSuite : public TestSuite
{
public:
  /**
   * @brief LocalizationTestSuite Constructor
   * @param planningModule Pointer to base planning module
   * @param config Configuration of this behavior
   */
  LocalizationTestSuite(
    PlanningModule* planningModule,
    const boost::shared_ptr<LocalizationTestSuiteConfig>& config);

  /**
   * @brief ~LocalizationTestSuite Destructor
   */
  ~LocalizationTestSuite() final {}

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

private:
  /**
   * @brief getBehaviorCast Returns the config casted as LocalizationTestSuiteConfigPtr
   */
  boost::shared_ptr<LocalizationTestSuiteConfig> getBehaviorCast();

  /**
   * @brief testSideLineLocalizationAction Tests localization for robot standing
   *   on sidelines
   */
  DECLARE_FSM(fsm, LocalizationTestSuite);

  /**
   * @brief OccupancyMapTest Tests the update of occupancy map while observing obstacles
   */
  DECLARE_FSM_STATE(LocalizationTestSuite, OccupancyMapTest, occupancyMapTest, onStart, onRun,);

  /**
   * @brief SideLineLocalization Tests localization for robot standing
   *   on sidelines
   */
  DECLARE_FSM_STATE(LocalizationTestSuite, SideLineLocalization, sideLineLocalization, onStart, onRun,);

  /**
   * @brief LocalizationLostState Tests localization for robot standing
   *   anywhere in the field with unknown position but with known half.
   */
  DECLARE_FSM_STATE(LocalizationTestSuite, LocalizationLostState, localizationLostState, onStart, onRun,);

  /**
   * @brief LocalizationPrediction Tests the prediction step of localization
   */
  DECLARE_FSM_STATE(LocalizationTestSuite, LocalizationPrediction, localizationPrediction, onStart, onRun,);

  /**
   * @brief LocalizationWithMovement Tests localization for robot starting from
   *   sidelines and moving to a goal spot
   */
  DECLARE_FSM_STATE(LocalizationTestSuite, LocalizationWithMovement, localizationWithMovement, onStart, onRun,);

  enum MBManagerIds {
    MOTION_1
  };
};

typedef boost::shared_ptr<LocalizationTestSuite> LocalizationTestSuitePtr;

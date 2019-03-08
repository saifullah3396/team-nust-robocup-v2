/**
 * @file PlanningBehaviors/TestSuite/Types/MotionTestSuite.h
 *
 * This file declares the class MotionTestSuite
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#pragma once

#include "PlanningModule/include/PlanningBehaviors/TestSuite/TestSuite.h"

struct MotionTestSuiteConfig;

/**
 * @class MotionTestSuite
 * @brief The class for testing Motion module functionality
 */
class MotionTestSuite : public TestSuite
{
public:
  /**
   * @brief MotionTestSuite Constructor
   * @param planningModule Pointer to base planning module
   * @param config Configuration of this behavior
   */
  MotionTestSuite(
    PlanningModule* planningModule,
    const boost::shared_ptr<MotionTestSuiteConfig>& config);

  /**
   * @brief ~MotionTestSuite Destructor
   */
  ~MotionTestSuite() final {}

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
   * @brief getBehaviorCast Returns the config casted as MotionTestSuiteConfigPtr
   */
  boost::shared_ptr<MotionTestSuiteConfig> getBehaviorCast();

  enum MBManagerIds {
    MOTION_1
  };

  //! Config of the motion being tested
  MBConfigPtr motionConfig;
};

typedef boost::shared_ptr<MotionTestSuite> MotionTestSuitePtr;

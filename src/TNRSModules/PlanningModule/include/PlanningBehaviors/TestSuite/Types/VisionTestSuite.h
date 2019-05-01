/**
 * @file PlanningBehaviors/TestSuite/Types/VisionTestSuite.h
 *
 * This file declares the class VisionTestSuite
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#pragma once

#include "PlanningModule/include/PlanningBehaviors/TestSuite/TestSuite.h"

struct VisionTestSuiteConfig;

/**
 * @class VisionTestSuite
 * @brief The class for testing vision module functionality
 */
class VisionTestSuite : public TestSuite
{
public:
  /**
   * @brief VisionTestSuite Constructor
   * @param planningModule Pointer to base planning module
   * @param config Configuration of this behavior
   * @param name Behavior name
   */
  VisionTestSuite(
    PlanningModule* planningModule,
    const boost::shared_ptr<VisionTestSuiteConfig>& config);

  /**
   * @brief ~VisionTestSuite Destructor
   */
  ~VisionTestSuite() final {}

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
   * @brief getBehaviorCast Returns the config casted as VisionTestSuiteConfigPtr
   */
  boost::shared_ptr<VisionTestSuiteConfig> getBehaviorCast();

  void logImages(); ///< Test for for logging images, this is used for camera calibration
  void testSegmentation(); ///< Test for region segmentation
  void testFieldExtraction(); ///< Test for field extraction
  void testGoalExtraction(); ///< Test for goal extraction
  void testBallExtraction(); ///< Test for ball extraction
  void testRobotExtraction(); ///< Test for robot extraction
  void testLinesExtraction(); ///< Test for lines extraction
  void testFieldProjection(); ///< Test for lines extraction
  void testAll(); ///< Test for all vision modules
};

typedef boost::shared_ptr<VisionTestSuite> VisionTestSuitePtr;

/**
 * @file PlanningModule/include/PlanningBehaviors/TestSuite/TestSuite.h
 *
 * This file declares the class TestSuite.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017
 */

#pragma once

#include "PlanningModule/include/PlanningBehavior.h"

/**
 * @class TestSuite
 * @brief A base class for defining test sequences
 */
class TestSuite : public PlanningBehavior
{

public:
  /**
   * @brief TestSuite Constructor
   * @param planningModule Pointer to base planning module
   * @param config Configuration of this behavior
   * @param name Behavior name
   */
  TestSuite(
    PlanningModule* planningModule,
    const boost::shared_ptr<TestSuiteConfig>& config,
    const string& name = "TestSuite");

  /**
   * @brief ~TestSuite Destructor
   */
  virtual ~TestSuite() {}

  /**
   * @brief getType Returns its own child based on the given type
   *
   * @param planningModule Pointer to base planning module
   * @param cfg Config of the requested behavior
   *
   * @return boost::shared_ptr<TestSuite>
   */
  static boost::shared_ptr<TestSuite> getType(
    PlanningModule* planningModule, const BehaviorConfigPtr& cfg);

private:
  /**
   * Returns the cast of config as TestSuiteConfigPtr
   */
  boost::shared_ptr<TestSuiteConfig> getBehaviorCast();
};

typedef boost::shared_ptr<TestSuite> TestSuitePtr;

/**
 * @file PlanningBehaviors/Robocup/Types/RobocupPenalties.h
 *
 * This file declares the class RobocupPenalties.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017
 */

#pragma once

#include "PlanningModule/include/PlanningBehaviors/Robocup/Robocup.h"

class PenaltiesConfig;

/**
 * @class RobocupPenalties
 * @brief The class for defining the robocup penalties behavior
 */
class RobocupPenalties : public Robocup
{
public:
  /**
   * @brief RobocupPenalties Constructor
   *
   * @param planningModule: pointer to parent planning module
   * @param config: Configuration of this behavior
   */
  RobocupPenalties(
    PlanningModule* planningModule,
    const boost::shared_ptr<PenaltiesConfig>& config);

  /**
   * @brief ~RobocupPenalties Destructor
   */
  ~RobocupPenalties()
  {
  }

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
  void penaltyCfgAction();

  /**
   * Returns the config casted as RobocupPenaltiesConfigPtr
   */
  boost::shared_ptr<PenaltiesConfig> getBehaviorCast();
};

typedef boost::shared_ptr<RobocupPenalties> RobocupPenaltiesPtr;

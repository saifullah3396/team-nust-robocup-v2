/**
 * @file PlanningModule/PlanningBehaviors/Defender.h
 *
 * This file declares the class Defender.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 May 2017
 */

#pragma once

#include "PlanningModule/include/PlanningBehaviors/Robocup/Types/Soccer.h"

struct DefenderConfig;

/**
 * @class Defender
 * @brief Class for defining the robocup defender behavior.
 */
class Defender : public Soccer
{
public:
  /**
   * Constructor
   *
   * @param planningModule: pointer to parent planning module
   * @param config: Configuration of this behavior
   */
  Defender(
    PlanningModule* planningModule,
    const boost::shared_ptr<DefenderConfig>& config);

  /**
   * Default destructor for this class.
   */
  ~Defender()
  {
  }

  bool initiate() final;

private:
  /**
   * Returns the config casted as DefenderConfigPtr
   */
  boost::shared_ptr<DefenderConfig> getBehaviorCast();

  /**
   * Derived from robocup
   */
  Point2f findBallKickTarget();

protected:
  struct ReactDefender : public React
  {
    ReactDefender(Defender* bPtr) : React(bPtr) {}
    virtual void onRun() final;
  };
  //friend class Robocup;

  bool ballInRange() final;
};

typedef boost::shared_ptr<Defender> DefenderPtr;

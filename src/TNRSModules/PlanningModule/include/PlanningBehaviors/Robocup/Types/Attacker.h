/**
 * @file PlanningBehaviors/Robocup/Types/Attacker.h
 *
 * This file declares the class Attacker.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 May 2017
 */

#pragma once

#include "PlanningModule/include/PlanningBehaviors/Robocup/Types/Soccer.h"

struct AttackerConfig;

/**
 * @class Attacker
 * @brief Class for defining the robocup attacker behavior.
 */
class Attacker : public Soccer
{
public:
  /**
   * Constructor
   *
   * @param planningModule: pointer to parent planning module
   * @param config: Configuration of this behavior
   */
  Attacker(
    PlanningModule* planningModule,
    const boost::shared_ptr<AttackerConfig>& config);

  /**
   * Default destructor for this class.
   */
  ~Attacker()
  {
  }

  bool initiate() final;

private:
  /**
   * Returns the config casted as AttackerConfigPtr
   */
  boost::shared_ptr<AttackerConfig> getBehaviorCast();

protected:
  struct ReactAttacker : public React
  {
    ReactAttacker(Attacker* bPtr) : React(bPtr) {}
    virtual void onRun() final;
  };
  //friend class Robocup;

  bool ballInRange() final;
};

typedef boost::shared_ptr<Attacker> AttackerPtr;

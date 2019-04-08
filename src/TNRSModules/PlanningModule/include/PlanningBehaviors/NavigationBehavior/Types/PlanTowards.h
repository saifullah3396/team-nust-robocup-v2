/**
 * @file PlanningBehaviors/NavigationBehavior/Types/PlanTowards.h
 *
 * This file declares the class PlanTowards
 *
 * @author <A href="mailto:">Umaid Muhammad Zaffar</A>
 * @author <A href="mailto:">Marium Aslam</A>
 * @date 7 April 2019
 */

#pragma once

#include <memory>
#include "PlanningModule/include/PlanningBehaviors/NavigationBehavior/NavigationBehavior.h"
#include "Utils/include/DataHolders/RobotPose2D.h"
#include "Utils/include/DataHolders/Obstacle.h"

template <typename Scalar>
class VelocityInput;
template <typename Scalar>
class PotentialField2D;
struct PlanTowardsConfig;

/**
 * @class PlanTowards
 * @brief The behavior for planning movement from start position to goal position.
 */
class PlanTowards : public NavigationBehavior
{
public:
  /**
   * @brief PlanTowards Constructor
   * @param planningModule Pointer to base planning module
   * @param config Configuration of this behavior
   */
  PlanTowards(
    PlanningModule* planningModule,
    const boost::shared_ptr<PlanTowardsConfig>& config);

  /**
   * @brief ~PlanTowards Destructor
   */
  ~PlanTowards() final;

  bool initiate() final;
  void reinitiate(const boost::shared_ptr<BehaviorConfig>& cfg) final;
  void update() final;
  void finish() final;
  void loadExternalConfig() final {}

private:
  /**
   * * Returns the config casted as PlanTowardsConfigPtr.
   */
  boost::shared_ptr<PlanTowardsConfig> getBehaviorCast();

  void executeMotionAction() final;

  std::unique_ptr<VelocityInput<float>> velocityInput;
  std::unique_ptr<PotentialField2D<float>> potentialField2D;
};

typedef boost::shared_ptr<PlanTowardsConfig> PlanTowardsPtr;

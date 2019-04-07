#pragma once

#include "PlanningModule/include/PlanningBehaviors/NavigationBehavior/NavigationBehavior.h"
#include "Utils/include/DataHolders/RobotPose2D.h"
#include "Utils/include/DataHolders/Obstacle.h"

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
    PlanTowards(PlanningModule planningModule, const boost::shared_ptr<PlanTowardsConfig>& config);
    /**
     * @brief ~PlanTowards Destructor
     */
    ~PlanTowards() final{}

private:
    /**
     * * Returns the config casted as PlanTowardsConfigPtr.
     */
    boost::shared_ptr<PlanTowardsConfig> getBehaviorCast();

    bool initiate();
    bool reinitiate(PlanTowardsConfig* cfg);
    bool update();
    bool finish();
    bool loadExternalConfig();

    void executeMotionAction() override;

   /* bool velocityForPath(
      const RobotPose2D<Scalar>& robotPose,
      const RobotPose2D<Scalar>& goalPose,
      const vector<Obstacle<Scalar>>& obstacles);*/

    RobotPose2D<float> goal;

    VelocityInput<Scalar> velocity;

};

typedef boost::shared_ptr<PlanTowardsConfig> PlanTowardsPtr;

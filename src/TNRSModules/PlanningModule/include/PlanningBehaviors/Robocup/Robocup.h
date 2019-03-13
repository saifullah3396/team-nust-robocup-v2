/**
 * @file PlanningModule/PlanningBehaviors/Robocup.h
 *
 * This file declares the class Robocup.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017 
 */

#pragma once

#include "BehaviorManager/include/StateMachineMacros.h"
#include "PlanningModule/include/PlanningBehavior.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "BehaviorConfigs/include/PBConfigs/PBRobocupConfig.h"

enum class KeyFrameGetupTypes : unsigned int;
struct MBHeadControlConfig;

class Robocup : public PlanningBehavior
{
public:
  /**
   * Constructor
   *
   * @param planningModule: pointer to parent planning module
   * @param config: Configuration of this behavior
   * @param name: Name of this behavior
   */
  Robocup(
    PlanningModule* planningModule,
    const BehaviorConfigPtr& config,
    const string& name = "Robocup") :
    PlanningBehavior(planningModule, config, name),
    inFallRecovery(false),
    readyToGetup(false),
    waitForUnpenalise(false),
    penaliseMotion(false),
    moveTarget(RobotPose2D<float>(100, 100, 100)),
    ballMotionModel(BallMotionModel::DAMPED)
  {
  }

  /**
   * Default destructor for this class.
   */
  virtual
  ~Robocup()
  {
  }
  
  static boost::shared_ptr<Robocup> getType(
    PlanningModule* planningModule, const BehaviorConfigPtr& cfg);

private:
  boost::shared_ptr<PBRobocupConfig> getBehaviorCast();

protected:
  bool isLocalized();
  bool ballFound();
  bool otherRobotOnBall();
  void updateRobotData();
  bool robotIsPenalised();
  bool robotIsFalling();
  bool waitForPenalty();
  void setRobotIntention();
  void setRobotSuggestions();
  void fallenRobotAction();
  void fallRecoveryAction();
  void getupFront();
  void getupBack();
  void getupSit();
  void printGameData();
  void setNavigationConfig(
    const RobotPose2D<float>& target,
    const boost::shared_ptr<MBPostureConfig>& startPosture = boost::shared_ptr<MBPostureConfig>(),
    const boost::shared_ptr<MBPostureConfig>& endPosture = boost::shared_ptr<MBPostureConfig>());
  void resetLocalizer();
  bool getupFromGround(
    const KeyFrameGetupTypes& getupType,
    const StiffnessState& desStiffness,
    const unsigned& mbManagerId);

  //! Whether robot is supposed to get into penalised posture
  bool penaliseMotion;
  bool inFallRecovery;
  bool readyToGetup;
  //! Whether to wait to get unpenalised
  bool waitForUnpenalise;
  RobotPose2D<float> moveTarget;

  enum MBManagerIds {
    MOTION_1,
  };

  enum class BallMotionModel {
    damped,
    friction
  } ballMotionModel;
};

typedef boost::shared_ptr<Robocup> RobocupPtr;

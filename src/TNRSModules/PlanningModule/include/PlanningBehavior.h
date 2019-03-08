/**
 * @file PlanningModule/include/PlanningBehavior.h
 *
 * This file declares the class PlanningBehavior
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#pragma once

#include "BehaviorManager/include/Behavior.h"
#include "PlanningModule/include/PlanningBehaviorIds.h"
#include "PlanningModule/include/PlanningModule.h"
#include "Utils/include/DataHolders/PostureState.h"
#include "Utils/include/DataHolders/StiffnessState.h"

class SBConfig;
typedef boost::shared_ptr<SBConfig> SBConfigPtr;
class MBConfig;
typedef boost::shared_ptr<MBConfig> MBConfigPtr;

namespace PathPlannerSpace
{
  class PathPlanner;
  typedef boost::shared_ptr<PathPlanner> PathPlannerPtr;
}

/**
 * @class PlanningBehavior
 * @brief A base class for all types of planning behaviors
 */
class PlanningBehavior : public Behavior, public MemoryBase
{
public:
  /**
   * Constructor
   *
   * @param pModule: Pointer to the parent planning module
   * @param config: Behavior configuration
   * @param name: Behavior name
   */
  PlanningBehavior(
    PlanningModule* pModule,
    const BehaviorConfigPtr& config,
    const string& name = "Not assigned.") :
      Behavior(config, name),
      MemoryBase(pModule), 
      pModule(pModule), 
      sRequestTime(0.f),
      mRequestTime(0.f) 
    //penaliseMotion(false), 
    //waitForUnpenalise(false)
  {
    cycleTime = pModule->getPeriodMinMS() / 1000.f;
    mbIdOffset = 0;
  }
  
  /**
   * Destructor
   */
  virtual
  ~PlanningBehavior()
  {
  }
  
  void setMBIdOffset(const int& mbIdOffset) {
    this->mbIdOffset = mbIdOffset;
  }

protected:
  /**
   * Sends the requests to change robot posture and stiffness states to
   * the given ones
   * 
   * @param desPosture: Desired posture
   * @param desStiffness: Desired stiffness
   * @param mbManagerId: Motion behavior id
   * 
   * @return true if both are set successfully else false
   */
  bool setPostureAndStiffness(
    const PostureState& desPosture, 
    const StiffnessState& desStiffness,
    const unsigned& mbManagerId,
    const float& postureTime = 2.f);
  
  /**
   * Sends the request to kill the running static behavior
   */ 
  void killStaticBehavior();
  
  /**
   * Sends the request to kill the running motion behavior
   *
   * @param mbManagerId: Id of the motion behavior manager responsible
   *   for the motion behavior
   */ 
  void killMotionBehavior(const unsigned& mbManagerId);

  /**
   * Sends the request to kill all the running motion behaviors
   */
  void killAllMotionBehaviors();

  /**
   * @brief matchLastMotionRequest Returns true if the last motion request matches
   *   the given behavior id
   * @param mbId Motion behavior id
   * @return bool
   */
  bool matchLastMotionRequest(
    const unsigned& mbId);

  /**
   * @brief matchLastMotionRequest Returns true if the last motion request matches
   *   the given behavior id and type
   * @param mbId Motion behavior id
   * @param mbType Motion behavior sub-type type id
   * @return bool
   */
  bool matchLastMotionRequest(
    const unsigned& mbId, const unsigned& mbType);

  /**
   * @brief matchLastStaticRequest Returns true if the last static request matches
   *   the given behavior id and type
   * @param sbId Static behavior id
   * @param sbType Static behavior sub-type type id
   * @return bool
   */
  bool matchLastStaticRequest(
    const unsigned& sbId, const unsigned& sbType);

  /**
   * Returns true if a static behavior is running
   * 
   * @return bool
   */ 
  bool sbInProgress();
  
  /**
   * Returns true if a motion behavior is running
   * 
   * @return bool
   */ 
  bool mbInProgress();
  
  /**
   * Sets up a request to call a static behavior
   * 
   * @param config: Configuration of the required static behavior
   */ 
  void setupSBRequest(const SBConfigPtr& config);
  
  /**
   * Sets up a request to call a motion behavior
   * 
   * @param config: Configuration of the required motion behavior
   */ 
  void setupMBRequest(
    const unsigned& mbManagerId, const MBConfigPtr& config);

  /**
   * Returns true if a static or motion behavior requests has been
   * sent but not accepted
   * 
   * @return bool
   */ 
  bool requestInProgress();

  /**
   * Sets stiffness to zero and removes all motion
   * behaviors if chest button is pressed. This function is called in upper
   * level for fast and safe termination of robot motion.
   *
   * @return bool
   */
  bool shutdownCallBack();

  /**
   * @brief Returns the path planner object from base PlanningModule
   * @return PathPlannerPtr
   */
  PathPlannerSpace::PathPlannerPtr getPathPlanner();

  //! Base PlanningModule object pointer.
  PlanningModule* pModule;

  //! Ids of the motion behavior managers running
  vector<unsigned> mbManagerIds;
  
  //! Offset for motion behavior ids to be used
  int mbIdOffset;

  //! Configuration of the currently running motion behavior
  BehaviorConfigPtr lastMBConfig;

  //! Configuratoin of the currently running static behavior
  BehaviorConfigPtr lastSBConfig;

  //! Last motion behaviors requested
  BehaviorRequestPtr lastMotionRequest;

  //! Last static behaviors requested
  BehaviorRequestPtr lastStaticRequest;

private:
  /**
   * Checks whether the behavior is still in progress based on incoming
   * info
   * 
   * @param info: The behavior info
   * 
   * @return true if behavior is still running
   */ 
  bool behaviorInProgress(const BehaviorInfo& info);

  /**
   * Returns true if the given request has not been received. If 
   * received and accepted, the acceptedBehavior is assigned the config 
   * of the given request. In case of rejection, the request is deleted
   * 
   * @param req: Request to be checked
   * @param acceptedBehavior: The configuration of the accepted behavior
   * @param info: The feed back information about behavior
   * @param requestStartTime: The time this request was sent at
   * 
   * @return bools
   */ 
  bool requestInProgress(
    BehaviorRequestPtr& req,
    BehaviorConfigPtr& acceptedBehavior,
    const BehaviorInfo& feedback,
    const float& requestStartTime);

  //! Time taken by the motion request in progress
  float mRequestTime;

  //! Time taken by the static behavior request in progress
  float sRequestTime;
  
  //! Maximum allowed time for a motion or behavior request to be 
  //! processed
  static constexpr float maxRequestTimeout = 2.f;
};

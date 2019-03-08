/**
 * @file LocalizationModule/include/LocalizationRequest.h
 *
 * This file defines the class LocalizationRequest
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#pragma once

#include <opencv2/core/core.hpp>
#include "TeamNUSTSPL/include/TNSPLModuleIds.h"
#include "TNRSBase/include/ModuleRequest.h"
#include "Utils/include/SwitchRequest.h"
#include "Utils/include/DataHolders/PositionInput.h"
#include "Utils/include/DataHolders/Landmark.h"

/**
 * Types of request valid for LocalizationModule
 * 
 * @enum LocalizationRequestIds
 */ 
enum class LocalizationRequestIds {
  switchLocalization,
  switchParticleFilter,
  switchFieldMap,
  switchBallObstacle,
  initLocalizer,
  resetLocalizer,
  positionUpdate,
  knownLandmarksUpdate,
  unknownLandmarksUpdate
};

/**
 * @class LocalizationRequest
 * @brief A module request that can be handled by LocalizationModule
 */ 
class LocalizationRequest : public ModuleRequest 
{
public:
  /**
   * Constructor
   * 
   * @param id: Id of the control request
   */ 
  LocalizationRequest(const LocalizationRequestIds& id) :
    ModuleRequest((unsigned)TNSPLModules::localization, (unsigned)id)
  {
  }
};
typedef boost::shared_ptr<LocalizationRequest> LocalizationRequestPtr;

/**
 * @class SwitchLocalization
 * @brief A request to switch on and off the localization module
 */ 
struct SwitchLocalization : public LocalizationRequest, SwitchRequest
{
  /**
   * Constructor
   */ 
  SwitchLocalization(const bool& state) :
    LocalizationRequest(LocalizationRequestIds::switchLocalization),
    SwitchRequest(state)
  {
  }
};
typedef boost::shared_ptr<SwitchLocalization> SwitchLocalizationPtr;

/**
 * @class SwitchParticleFilter
 * @brief A request to switch on and off the particle filter update
 */
struct SwitchParticleFilter : public LocalizationRequest, SwitchRequest
{
  /**
   * Constructor
   */
  SwitchParticleFilter(const bool& state) :
    LocalizationRequest(LocalizationRequestIds::switchParticleFilter),
    SwitchRequest(state)
  {
  }
};
typedef boost::shared_ptr<SwitchParticleFilter> SwitchParticleFilterPtr;

/**
 * @class SwitchFieldMap
 * @brief A request to switch on and off the field map update
 */
struct SwitchFieldMap : public LocalizationRequest, SwitchRequest
{
  /**
   * Constructor
   */
  SwitchFieldMap(const bool& state) :
    LocalizationRequest(LocalizationRequestIds::switchFieldMap),
    SwitchRequest(state)
  {
  }
};
typedef boost::shared_ptr<SwitchFieldMap> SwitchFieldMapPtr;

/**
 * @class SwitchBallObstacle
 * @brief A request to switch on and off the ball to be used as obstacle
 */ 
struct SwitchBallObstacle : public LocalizationRequest, SwitchRequest
{
  /**
   * Constructor
   */ 
  SwitchBallObstacle(const bool& state) :
    LocalizationRequest(LocalizationRequestIds::switchBallObstacle),
    SwitchRequest(state)
  {
  }
};
typedef boost::shared_ptr<SwitchBallObstacle> SwitchBallObstaclePtr;

/**
 * @class InitiateLocalizer
 * @brief A request to initiate the localizer of the robot
 */
struct InitiateLocalizer : public LocalizationRequest
{
  /**
   * Constructor
   */
  InitiateLocalizer(const RobotPose2D<float>& state) :
    LocalizationRequest(LocalizationRequestIds::initLocalizer),
    state(state)
  {
  }

  RobotPose2D<float> state;
};
typedef boost::shared_ptr<InitiateLocalizer> InitiateLocalizerPtr;

/**
 * @class ResetLocalizer
 * @brief A request to reset localizer of the robot
 */ 
struct ResetLocalizer : public LocalizationRequest
{
  /**
   * Constructor
   */ 
  ResetLocalizer() :
    LocalizationRequest(LocalizationRequestIds::resetLocalizer)
  {
  }
};
typedef boost::shared_ptr<ResetLocalizer> ResetLocalizerPtr;

/**
 * @class PositionUpdate
 * @brief A request to update the position of the robot
 */ 
struct PositionUpdate : public LocalizationRequest
{
  /**
   * Constructor
   */ 
  PositionUpdate(const PositionInput<float>& input) :
    LocalizationRequest(LocalizationRequestIds::positionUpdate),
    input(input)
  {
  }
  
  PositionInput<float> input;
};

typedef boost::shared_ptr<PositionUpdate> PositionUpdatePtr;
typedef boost::shared_ptr<KnownLandmark<float> > KnownLandmarkPtr;
typedef boost::shared_ptr<UnknownLandmark<float> > UnknownLandmarkPtr;

/**
 * @class KnownLandmarksUpdate
 * @brief A request to update the observed known landmarks for the localizer
 */ 
struct KnownLandmarksUpdate : public LocalizationRequest
{
  /**
   * Constructor
   */ 
  KnownLandmarksUpdate(const vector<KnownLandmarkPtr>& landmarks) :
    LocalizationRequest(LocalizationRequestIds::knownLandmarksUpdate),
    landmarks(landmarks)
  {
  }
  
  vector<KnownLandmarkPtr> landmarks;
};
typedef boost::shared_ptr<KnownLandmarksUpdate> KnownLandmarksUpdatePtr;

/**
 * @class UnknownLandmarksUpdate
 * @brief A request to update the observed unknown landmarks for the localizer
 */ 
struct UnknownLandmarksUpdate : public LocalizationRequest
{
  /**
   * Constructor
   */ 
  UnknownLandmarksUpdate(const vector<UnknownLandmarkPtr>& landmarks) :
    LocalizationRequest(LocalizationRequestIds::unknownLandmarksUpdate),
    landmarks(landmarks)
  {
  }
  
  vector<UnknownLandmarkPtr> landmarks;
};
typedef boost::shared_ptr<UnknownLandmarksUpdate> UnknownLandmarksUpdatePtr;

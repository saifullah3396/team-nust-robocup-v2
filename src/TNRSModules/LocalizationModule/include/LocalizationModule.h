/**
 * @file LocalizationModule/include/LocalizationModule.h
 *
 * This file declares a class for robot localization and mapping.
 * All the functions and algorithms for robot state estimation and
 * pose determination will be defined under this module.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 11 Feb 2017
 */

#pragma once

#include "TNRSBase/include/BaseIncludes.h"

class FieldMap;
class ParticleFilter;

/**
 * @class LocalizationModule
 * @brief The class for robot localization and mapping
 */
class LocalizationModule : public BaseModule
{
  DECLARE_INPUT_CONNECTOR(
    localizationThreadPeriod,
    gameData,
    upperCamInFeet,
    lowerCamInFeet,
    planningState,
    obstaclesObs,
    goalInfo,
    ballInfo,
    worldBallInfo,
    robotOnSideLine,
    localizeWithLastKnown,
    robotInMotion
  )
  DECLARE_OUTPUT_CONNECTOR(
    robotPose2D,
    lastKnownPose2D,
    occupancyMap,
    robotLocalized,
    positionConfidence,
    sideConfidence,
    fieldWidth,
    fieldHeight
  )
public:
  /**
   * Constructor
   *
   * @param processingModule: Pointer to base class which is
   *   in this case the ProcessingModule.
   */
  LocalizationModule(void* processingModule);

  /**
   * Destructor
   */
  ~LocalizationModule()
  {
    delete inputConnector;
    delete outputConnector;
  }

  /**
   * Derived from BaseModule
   */
  void init();

  /**
   * Derived from BaseModule
   */
  void mainRoutine();

  /**
   * Derived from BaseModule
   */
  void handleRequests();

  /**
   * Derived from BaseModule
   */
  void initMemoryConn();

  /**
   * Derived from BaseModule
   */
  void setThreadPeriod();

  /**
   * @brief getParticleFilter Gets the particle filter object
   * @return
   */
  boost::shared_ptr<ParticleFilter> getParticleFilter() { return particleFilter; }

private:
  //! Whether to perform processing related to robot localization
  bool runLocalization = {true};

  //! Whether to update the particle filter
  bool updateFilter = {true};

  //! Whether to update the field map
  bool updateMap = {true};

  //! Particle filter object
  boost::shared_ptr<ParticleFilter> particleFilter;

  //! Field map object
  boost::shared_ptr<FieldMap> fieldMap;
};

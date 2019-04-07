/**
 * @file LocalizationModule/include/LandmarkDefinitions.h
 *
 * This file declares the landmark positions on the field.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 22 Aug 2017  
 */

#pragma once

#include "Utils/include/DataHolders/RobotPose2D.h"
#include "LocalizationModule/include/FieldLandmarkIds.h"

constexpr float fieldMaxX = 4.5f;
constexpr float fieldMidX = 0.f;
constexpr float fieldMinX = -4.5f;
constexpr float fieldMaxY = 3.f;
constexpr float fieldMidY = 0.f;
constexpr float fieldMinY = -3.f;
constexpr float goalPostX = 4.5f;
constexpr float goalPostY = 0.8f;
constexpr float penaltyBoxMaxX = 4.5f;
constexpr float penaltyBoxMinX = 3.9f;
constexpr float penaltyBoxMidX = (penaltyBoxMinX + penaltyBoxMaxX) / 2;
constexpr float penaltyBoxMaxY = 1.1f;
constexpr float penaltyBoxMinY = -1.1f;
constexpr float penaltyMarkerX = 3.2f;
constexpr float penaltyMarkerY = 0.f;

static const RobotPose2D<float> landmarksOnField[NUM_LANDMARKS] =
{
  RobotPose2D<float>(goalPostX, goalPostY, -M_PI_2), ///<Left top goal post
  RobotPose2D<float>(goalPostX, -goalPostY, -M_PI_2), ///<Right top goal post
  RobotPose2D<float>(-goalPostX, goalPostY, M_PI_2), ///<Left bottom goal post
  RobotPose2D<float>(-goalPostX, -goalPostY, M_PI_2), ///<Right bottom goal post
  RobotPose2D<float>(fieldMidX, fieldMaxY, -M_PI_2), ///<Middle T L
  RobotPose2D<float>(fieldMidX, fieldMinY, M_PI_2), ///<Middle T R
  RobotPose2D<float>(fieldMaxX, penaltyBoxMaxY, M_PI), ///<Box T L 1
  RobotPose2D<float>(fieldMaxX, penaltyBoxMinY, M_PI), ///<Box T R 1
  RobotPose2D<float>(fieldMinX, penaltyBoxMaxY, 0.f), ///<Box T L 1
  RobotPose2D<float>(fieldMinX, penaltyBoxMinY, 0.f), ///<Box T R 1
  RobotPose2D<float>(fieldMaxX, fieldMaxY, 5 / 4.f * M_PI), ///<Corner 1
  RobotPose2D<float>(fieldMaxX, fieldMinY, 3 / 4.f * M_PI), ///<Corner 2
  RobotPose2D<float>(fieldMinX, fieldMaxY, -M_PI / 4.f), ///<Corner 3
  RobotPose2D<float>(fieldMinX, fieldMinY, M_PI / 4.f), ///<Corner 4
  RobotPose2D<float>(penaltyBoxMinX, penaltyBoxMaxY, -M_PI / 4.f), ///<Box Corner L 1
  RobotPose2D<float>(penaltyBoxMinX, penaltyBoxMinY, M_PI / 4.f), ///<Box Corner R 1
  RobotPose2D<float>(-penaltyBoxMinX, penaltyBoxMaxY, 5 / 4.f * M_PI), ///<Box Corner L 1
  RobotPose2D<float>(-penaltyBoxMinX, penaltyBoxMinY, 3 / 4.f * M_PI), ///<Box Corner R 1
  RobotPose2D<float>(fieldMaxX, penaltyBoxMaxY, M_PI), ///<Box Corner L Back 1, same as Box T
  RobotPose2D<float>(fieldMaxX, penaltyBoxMinY, M_PI), ///<Box Corner R Back 1, same as Box T
  RobotPose2D<float>(fieldMinX, penaltyBoxMaxY, 0.f), ///<Box Corner L Back 1, same as Box T
  RobotPose2D<float>(fieldMinX, penaltyBoxMinY, 0.f), ///<Box Corner R Back 1, same as Box T
  RobotPose2D<float>(fieldMidX, fieldMidY, M_PI_2), ///<Center
  RobotPose2D<float>(penaltyMarkerX, penaltyMarkerY, -M_PI_2), ///<Penalty mark 1
  RobotPose2D<float>(-penaltyMarkerX, penaltyMarkerY, M_PI_2), ///<Penalty mark 2
};

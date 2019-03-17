/**
 * @file BehaviorConfigs/include/GBConfigs/GBLedsConfig.h
 *
 * This file defines the class GBLedsConfig and its childs
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#pragma once

#include "BehaviorConfigs/include/GBConfigs/GBConfig.h"
#include "Utils/include/HardwareIds.h"
#include "Utils/include/DataUtils.h"
#include "Utils/include/DebugUtils.h"

/**
 * @struct MBLedsConfig
 * @brief The leds behavior configuration
 */
struct GBLedsConfig : GBConfig
{
	/**
	 * Constructor
	 * 
	 * @param inToReach: Required values of intensities to reach
	 * @param timeToReacIn: Time in which intensities are required to reach
	 *   the given values
	 * @param type: Type of the led behavior
	 */
  GBLedsConfig(
    const vector<float>& inToReach,
    const float& timeToReachIn,
    const GBLedsTypes& type = GBLedsTypes::directLeds) :
    GBConfig(GBIds::ledsModule, 45.f, (int)type),
    inToReach(inToReach),
    timeToReachIn(timeToReachIn)
  {
  }

	/**
	 * Constructor
	 * 
	 * @param groupId: Led group whose intensity is to be changed
	 * @param requiredIn: Required value of intensity
	 * @param timeToReacIn: Time in which intensities are required to reach
	 *   the given values
	 * @param type: Type of the led behavior
	 */
  GBLedsConfig(
    const LedGroups& groupId,
    const float& requiredIn,
    const float& timeToReachIn,
    const GBLedsTypes& type = GBLedsTypes::directLeds) :
    GBConfig(GBIds::ledsModule, 3.f, (int)type),
    timeToReachIn(timeToReachIn)
  {
    LedActuators groupStart, groupSize;
    getGroup(groupStart, groupSize, groupId);
    inToReach = vector<float>(static_cast<int>(LedActuators::count), NAN);
    for (
      size_t i = static_cast<unsigned>(groupStart);
      i < static_cast<unsigned>(groupStart) + static_cast<unsigned>(groupSize);
      ++i)
    {
      inToReach[i] = requiredIn;
    }
  }

	/**
	 * Constructor
	 * 
	 * @param groupId: Led group whose intensity is to be changed
	 * @param requiredIn: Required value of intensity
	 * @param timeToReacIn: Time in which intensities are required to reach
	 *   the given values
	 * @param bgr: The bgr values required for the given group
	 * @param type: Type of the led behavior
	 */
  GBLedsConfig(
    const LedGroups& groupId,
    const float& requiredIn,
    const float& timeToReachIn,
    const vector<unsigned>& bgr = vector<unsigned>(3),
    const GBLedsTypes& type = GBLedsTypes::directLeds) :
    GBConfig(GBIds::ledsModule, 3.f, (int)type),
    timeToReachIn(timeToReachIn)
  {
    ASSERT(bgr.size() == 3) // For b, g, and r colors
    ASSERT(// RGB only supported for these led groups
      groupId == LedGroups::lFace ||
      groupId == LedGroups::rFace ||
      groupId == LedGroups::chest ||
      groupId == LedGroups::lFoot ||
      groupId == LedGroups::rFoot);
    LedActuators groupStart, groupSize;
    getGroup(groupStart, groupSize, groupId);
    inToReach = vector<float>(static_cast<unsigned>(LedActuators::count), NAN);
    unsigned colorIndex = 2;
    float color;
    for (
      size_t i = static_cast<unsigned>(groupStart);
      i < static_cast<unsigned>(groupStart) + static_cast<unsigned>(groupSize);
      ++i)
    {
      color = bgr[colorIndex] / 255;
      inToReach[i] = requiredIn * color;
      if (!((i+1) % (static_cast<unsigned>(groupSize) / 3))) { //3 for blue, green, red
        colorIndex--;
      }
    }
  }

	/**
	 * Sets the start and size of the group of leds based on its id.
	 * 
	 * @param groupStart: Start index
	 * @param groupSize: Number of leds in the group
	 * @param groupId: Id of the group
	 */
  void getGroup(
    LedActuators& groupStart,
    LedActuators& groupSize,
    const LedGroups& groupId)
  {
    if (groupId == LedGroups::head) {
      groupStart = LedActuators::headLedRearLeft_0_Actuator;
      groupSize = LedActuators::nHeadLed;
    } else if (groupId == LedGroups::lFace) {
      groupStart = LedActuators::faceLedRedLeft_0_degActuator; // OK
      groupSize = LedActuators::nFaceLLed; // OK
    } else if (groupId == LedGroups::rFace) {
      groupStart = LedActuators::faceLedRedRight_0_degActuator; // OK
      groupSize = LedActuators::nFaceRLed; // OK
    } else if (groupId == LedGroups::lEar) {
      groupStart = LedActuators::earsLedLeft_0_degActuator; // OK
      groupSize = LedActuators::nEarLLed; // OK
    } else if (groupId == LedGroups::rEar) {
      groupStart = LedActuators::earsLedRight_0_degActuator; // OK
      groupSize = LedActuators::nEarRLed; // OK
    } else if (groupId == LedGroups::chest) {
      groupStart = LedActuators::chestBoardLedRedActuator;
      groupSize = LedActuators::nChestLed;
    } else if (groupId == LedGroups::lFoot) {
      groupStart = LedActuators::lFootLedRedActuator;
      groupSize = LedActuators::nLFeetLed ;
    } else if (groupId == LedGroups::rFoot) {
      groupStart = LedActuators::rFootLedRedActuator;
      groupSize = LedActuators::nRFeetLed;
    }
  }

	/**
	 * Validates the given configuration parameters
	 */ 
  void validate() {
    if (timeToReachIn <= 0.f || // Undefined time given
        inToReach.size() != static_cast<int>(LedActuators::count))
    {
      throw
        BConfigException(
          this,
          "Invalid behavior configuration parameters passed.",
          false
        );
    }
  }

	//! Time to reach the given led intensities
  float timeToReachIn;
  
  //! Intensities to reach for each led
  vector<float> inToReach;
};

typedef boost::shared_ptr<GBLedsConfig> GBLedsConfigPtr;

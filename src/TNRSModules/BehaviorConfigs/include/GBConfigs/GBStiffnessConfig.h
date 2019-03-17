/**
 * @file BehaviorConfigs/include/GBConfigs/GBStiffnessConfig.h
 *
 * This file defines the class GBStiffnessConfig and its childs
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#pragma once

#include "GBModule/include/StiffnessModule/StiffnessDefinitions.h"
#include "BehaviorConfigs/include/GBConfigs/GBConfig.h"
#include "Utils/include/HardwareIds.h"
#include "Utils/include/DataHolders/StiffnessState.h"

/**
 * @struct MBStiffnessConfig
 * @brief The stiffness behavior configuration
 */
struct GBStiffnessConfig : GBConfig
{
	/**
	 * Constructor
	 */
  GBStiffnessConfig() :
    GBConfig(GBIds::stiffnessModule, 3.f, (int)GBStiffnessTypes::stiffnessInterp),
    sToReach(vector<float>(toUType(Joints::count), 1.f)),
    timeToReachS(1.0f),
    targetState(StiffnessState::unknown)
  {
  }

	/**
	 * Constructor
	 * 
	 * @param sToReach: Stiffnesses to reach
	 * @param timeToReachS: Time in which stiffness are required to reach
	 *   the given values
	 * @param type: Type of the stiffness behavior
	 */
  GBStiffnessConfig(
    const vector<float>& sToReach,
    const float& timeToReachS,
    const GBStiffnessTypes& type = GBStiffnessTypes::stiffnessInterp) :
    GBConfig(GBIds::stiffnessModule, 3.f, (int)type),
    sToReach(sToReach),
    timeToReachS(timeToReachS),
    targetState(StiffnessState::unknown)
  {
  }
  
	/**
	 * Constructor
	 * 
	 * @param state: Stiffness state to reach
	 * @param timeToReachS: Time in which stiffness are required to reach
	 *   the given values
	 * @param type: Type of the stiffness behavior
	 */
  GBStiffnessConfig(
    const StiffnessState& state,
    const float& timeToReachS = 1.f,
    const GBStiffnessTypes& type = GBStiffnessTypes::stiffnessInterp) :
    GBConfig(GBIds::stiffnessModule, 3.f, (int)type),
    targetState(state),
    timeToReachS(timeToReachS)
  {
    if (state == StiffnessState::unknown)
      StiffnessState::robocup; // Set robocup as default
    sToReach =
      vector<float>(
        stiffnessDefinitions[toUType(state)],
        stiffnessDefinitions[toUType(state)] +
        sizeof(stiffnessDefinitions[toUType(state)]) /
        sizeof(stiffnessDefinitions[toUType(state)][0])
      );
  }
  
	/**
	 * Constructor
	 * 
	 * @param chainId: Robot chain whose stiffness is to be changed
	 * @param requiredS: Required value of stiffness
	 * @param timeToReachS: Time in which stiffness are required to reach
	 *   the given values
	 * @param type: Type of the stiffness behavior
	 */
  GBStiffnessConfig(
    const LinkChains& chainId,
    const float& requiredS,
    const float& timeToReachS,
    const GBStiffnessTypes& type = GBStiffnessTypes::stiffnessInterp) :
    GBConfig(GBIds::stiffnessModule, 3.f, (int)type),
    timeToReachS(timeToReachS),
    targetState(StiffnessState::unknown)
  {
    unsigned chainStart;
    unsigned chainSize;
    if (chainId == LinkChains::head) {
      chainStart = toUType(Joints::headYaw);
      chainSize = toUType(HardwareIds::nHead);
    } else if (chainId == LinkChains::lArm) {
      chainStart = toUType(Joints::lShoulderPitch);
      chainSize = toUType(HardwareIds::nLArm);
    } else if (chainId == LinkChains::rArm) {
      chainStart = toUType(Joints::rShoulderPitch);
      chainSize = toUType(HardwareIds::nRArm);
    } else if (chainId == LinkChains::lLeg) {
      chainStart = toUType(Joints::lHipYawPitch);
      chainSize = toUType(HardwareIds::nLLeg);
    } else if (chainId == LinkChains::rLeg) {
      chainStart = toUType(Joints::rHipYawPitch);
      chainSize = toUType(HardwareIds::nRLeg);
    }
    sToReach = vector<float>(toUType(Joints::count), NAN);
    for (size_t i = chainStart; i < chainStart + chainSize; ++i)
    {
      sToReach[i] = requiredS;
    }
  }

	/**
	 * Validates the given configuration parameters
	 */ 
  void validate() {
    if (timeToReachS <= 0.f || // Undefined time given
        toUType(targetState) >=
        toUType(StiffnessState::count) ||
        sToReach.size() != toUType(Joints::count))
    {
      throw
        BConfigException(
          this,
          "Invalid behavior configuration parameters passed.",
          false
        );
    }
  }

  //! Time to reach the given stiffnesses
  float timeToReachS;
  
  //! Stiffnesses to reach for each joint
  vector<float> sToReach;
  
  //! Target stiffness state
  StiffnessState targetState;
};

typedef boost::shared_ptr<GBStiffnessConfig> GBStiffnessConfigPtr;

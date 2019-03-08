/**
 * @file Utils/include/Behaviors/MBConfigs/MBTeleopConfig.h
 *
 * This file defines the struct MBTeleopConfig and its childs
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#include "Utils/include/Behaviors/MBConfigs/MBTeleopConfig.h"
#include "Utils/include/HardwareIds.h"

DEFINE_BEHAVIOR_CONFIG(
  MBTeleopConfig, MBConfig, MBTeleopConfigPtr,
  (MBTeleopTypes, teleopJoints, TeleopJointsConfig),
)

void TeleopJointsConfig::validate()
{
  if (jointCommands.size() != toUType(Joints::count))
  {
    throw
      BConfigException(
        this,
        "Invalid behavior configuration parameters passed.",
        false,
        EXC_INVALID_BCONFIG_PARAMETERS
      );
  }
}

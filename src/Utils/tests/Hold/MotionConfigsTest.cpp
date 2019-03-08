#include <iostream>
#include "Utils/include/Behaviors/MBConfigs/MBBalanceConfig.h"
#include "Utils/include/Behaviors/MBConfigs/MBBallThrowConfig.h"
#include "Utils/include/Behaviors/MBConfigs/MBDiveConfig.h"
#include "Utils/include/Behaviors/MBConfigs/MBGetupConfig.h"
#include "Utils/include/Behaviors/MBConfigs/MBHeadControlConfig.h"
#include "Utils/include/Behaviors/MBConfigs/MBKickConfig.h"
#include "Utils/include/Behaviors/MBConfigs/MBMovementConfig.h"
#include "Utils/include/Behaviors/MBConfigs/MBPostureConfig.h"

#define PRINT_JSON_CONFIG(NAME) \
  cout << #NAME << boost::make_shared<NAME>()->getJson() << endl;

int main()
{
  PRINT_JSON_CONFIG(MBPostureConfig);
  PRINT_JSON_CONFIG(JSOImpKickConfig);
  PRINT_JSON_CONFIG(JSE2DImpKickConfig);
  PRINT_JSON_CONFIG(MBBallThrowConfig);
  PRINT_JSON_CONFIG(MPComControlConfig);
  PRINT_JSON_CONFIG(PIDComControlConfig);
  PRINT_JSON_CONFIG(ZmpControlConfig);
  PRINT_JSON_CONFIG(NaoqiFootstepsConfig);
  PRINT_JSON_CONFIG(NaoqiMoveTowardConfig);
  PRINT_JSON_CONFIG(HeadTargetSearchConfig);
  PRINT_JSON_CONFIG(HeadTargetTrackConfig);
  PRINT_JSON_CONFIG(KFMDiveConfig);
  PRINT_JSON_CONFIG(KFMGetupConfig);
  PRINT_JSON_CONFIG(KFMGetupConfig);
}

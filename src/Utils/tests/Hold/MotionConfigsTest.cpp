#include <iostream>
#include "BehaviorConfigs/include/MBConfigs/MBBalanceConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBBallThrowConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBDiveConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBGetupConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBHeadControlConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBKickConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBMovementConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBPostureConfig.h"

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

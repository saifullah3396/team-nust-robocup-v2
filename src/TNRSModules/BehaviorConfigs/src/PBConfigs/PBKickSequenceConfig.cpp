/**
 * @file TNRSModules/BehaviorConfigs/src/PBConfigs/PBKickSequenceConfig.cpp
 *
 * This file defines the struct PBKickSequenceConfig and its childs
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#include "BehaviorConfigs/include/PBConfigs/PBKickSequenceConfig.h"

DEFINE_BEHAVIOR_CONFIG(
  PBKickSequenceConfig, PBConfig, PBKickSequenceConfigPtr,
  (PBKickSequenceTypes, ballIntercept, BallInterceptConfig),
  (PBKickSequenceTypes, findAndKick, FindAndKickConfig),
);

void BallInterceptConfig::init() {}
void BallInterceptConfig::validate() {}
void FindAndKickConfig::init() {}
void FindAndKickConfig::validate() {}

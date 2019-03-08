/**
 * @file MotionModule/include/KickModule/KickFootMap.h
 *
 * This file defines the kick foot mapping based on ball y coordinate and
 * target angle
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 15 May 2017
 */

#pragma once

#include "Utils/include/HardwareIds.h"
#include "MotionModule/include/MTypeHeader.h"

const MType kickFootMap[14][5] =
{
  { toUType(LinkChains::lLeg), 0.115, 0.125,   30,  90 },
  { toUType(LinkChains::lLeg), 0.1, 0.115,   0,  45 },
  { toUType(LinkChains::lLeg), 0.06, 0.1,   0,  15 },
  { toUType(LinkChains::lLeg), 0.04,   0.06,  -45, 45 },
  { toUType(LinkChains::lLeg), 0.025,  0.04,   -45, 0 },
  { toUType(LinkChains::lLeg), 0.0,  0.025,   -45, 0 },
  { toUType(LinkChains::lLeg), -0.025,   0.0, -90, -75 },
  { toUType(LinkChains::rLeg), 0.0,   0.025, 75, 90 },
  { toUType(LinkChains::rLeg), -0.025,  0.0,   0, 45 },
  { toUType(LinkChains::rLeg), -0.04,  -0.025,   0, 45 },
  { toUType(LinkChains::rLeg), -0.06,   -0.04,  -45, 45 },
  { toUType(LinkChains::rLeg), -0.1, -0.06,   0,  -15 },
  { toUType(LinkChains::rLeg), -0.115, -0.1,   -45, 0},
  { toUType(LinkChains::rLeg), -0.125, -0.115,   -90,  -30 }
};

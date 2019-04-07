/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
 
#include <stdio.h>
#include <string>
#include <vector>
#include "Utils/include/JsonUtils.h"
#include "Utils/include/HardwareIds.h"

namespace team_nust_visualizer_plugins
{

#define NUM_ROBOT_NAMES 9

enum TNColors : unsigned int
{
  white = 0,
  black,
  green,
  blue,
  red,
  yellow,
  count
};

static const std::string tn_colors[toUType(TNColors::count)] {
  "White",
  "Black",
  "Green",
  "Blue",
  "Red",
  "Yellow"
};

static const std::string slider_names[6] {
  "Ymin:", "Umin:", "Vmin:",
  "Ymax:", "Umax:", "Vmax:"
};

static const std::string camera_names[toUType(CameraId::count)] {
  "visionTop",
  "visionBottom"
};

static const std::string robot_names[NUM_ROBOT_NAMES] 
{
 "Sim",
 "Nu-11",
 "Nu-12",
 "Nu-13",
 "Nu-14",
 "Nu-15",
 "Nu-16",
 "Nu-17",
 "Nu-18"
};

static const unsigned n_color_tables[toUType(TNColors::count)] {
  2,
  1,
  3,
  1,
  1,
  1
};

} // end namespace team_nust_visualizer_plugins

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
 
#pragma once

#include <std_msgs/String.h>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <team_nust_msgs/JointInfo.h>
#include <team_nust_msgs/SensorState.h>
#include <team_nust_msgs/BehaviorInfo.h>

class QTableWidget;
class QTableWidgetItem;

#define MAX_TEXT_BUFFER 100
#define DECLARE_SENSOR_CALLBACK_FUNC(Name) \
  void Name(const team_nust_msgs::SensorState::ConstPtr& info);

namespace team_nust_visualizer_plugins
{
  
class SensorsInfo: public rviz::Panel
{
Q_OBJECT
public:
  SensorsInfo(QWidget* parent = 0);
  ~SensorsInfo() {}
  
private:
  void updateJointStates(const sensor_msgs::JointState::ConstPtr& state);
  void updateJointsInfo(const team_nust_msgs::JointInfo::ConstPtr& info);
  DECLARE_SENSOR_CALLBACK_FUNC(updateHandSensors);
  DECLARE_SENSOR_CALLBACK_FUNC(updateTouchSensors);
  DECLARE_SENSOR_CALLBACK_FUNC(updateSwitchSensors);
  DECLARE_SENSOR_CALLBACK_FUNC(updateBatterySensors);
  DECLARE_SENSOR_CALLBACK_FUNC(updateInertialSensors);
  DECLARE_SENSOR_CALLBACK_FUNC(updateSonarSensors);
  DECLARE_SENSOR_CALLBACK_FUNC(updateFsrSensors);
  DECLARE_SENSOR_CALLBACK_FUNC(updateLedSensors);
  
  // The current name of the output topic.
  QString input_topic_;
  
  // Data table
  std::vector<QTableWidget*> data_tables;
  
  // Subscriber for joints state topic
  ros::Subscriber joint_states_subscriber;
  ros::Subscriber joints_info_subscriber;
  std::vector<ros::Subscriber> sensor_subscribers;
  
  // Total number of tables
  unsigned n_tables;
  
  // The ROS node handle.
  ros::NodeHandle nh_;
};

} // team_nust_visualizer_plugins

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
#include <team_nust_msgs/TeamNUSTState.h>
#include <team_nust_msgs/LocalizationState.h>
#include <team_nust_msgs/BehaviorInfo.h>

class QLabel;
class QTableWidget;
class QTableWidgetItem;

#define MAX_TEXT_BUFFER 100

namespace team_nust_visualizer_plugins
{
  
class RobotState: public rviz::Panel
{
Q_OBJECT
public:
  RobotState(QWidget* parent = 0);
  ~RobotState() {}
  
private:
  void updateTeamNUSTState(const team_nust_msgs::TeamNUSTState::ConstPtr& state);
  void setModulesState(const team_nust_msgs::TeamNUSTState::ConstPtr& state);
  void setRobotState(const team_nust_msgs::TeamNUSTState::ConstPtr& state);
  QString getBehaviorContent(const team_nust_msgs::BehaviorInfo::ConstPtr& b_info);
  void setGameState(const team_nust_msgs::TeamNUSTState::ConstPtr& state);
  void updateLocalizerState(const team_nust_msgs::LocalizationState::ConstPtr& state);
  void updatePBInfo(const team_nust_msgs::BehaviorInfo::ConstPtr& b_info);
  void updateMBInfo(const team_nust_msgs::BehaviorInfo::ConstPtr& b_info);
  void updateSBInfo(const team_nust_msgs::BehaviorInfo::ConstPtr& b_info);
   
  // The current name of the output topic.
  QString input_topic_;
  
  // Data table
  std::vector<QTableWidget*> data_tables;
  
  // Connection status
  QLabel* connectionStatus;
  
  // Subscriber for robot state topic
  ros::Subscriber tnrs_state_subscriber;
  
  // Subscriber for behaviors info topic
  ros::Subscriber pb_info_subscriber;
  ros::Subscriber mb_info_subscriber;
  ros::Subscriber sb_info_subscriber;
  
  // Subscriber for localizer state
  ros::Subscriber localizer_state_subscriber;
  
  // The ROS node handle.
  ros::NodeHandle nh_;
};

} // team_nust_visualizer_plugins

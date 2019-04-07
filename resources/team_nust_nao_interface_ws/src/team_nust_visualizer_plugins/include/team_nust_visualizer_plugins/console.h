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

class QLineEdit;
class QPlainTextEdit;

#define MAX_TEXT_BUFFER 100

namespace team_nust_visualizer_plugins
{
  
class Console: public rviz::Panel
{
Q_OBJECT
public:
  Console(QWidget* parent = 0);
  ~Console() {}

  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;
  
public Q_SLOTS:
  void setTopic( const QString& topic );

protected Q_SLOTS:
  void displayMessage();
  void updateTopic();
  void publishCommand();
  
private:
  void handleTopic(const std_msgs::String::ConstPtr& msg);

  // One-line text editor for entering the outgoing ROS topic name.
  QLineEdit* input_topic_editor_;
  
  // One-line text editor for entering commands
  QLineEdit* output_cmd_editor_;
  
  // The current name of the output topic.
  QString input_topic_;
  
  // Number of text lines displayed
  int n_text_lines_;
  
  // Text display box
  QPlainTextEdit* text_box_;
  
  // Message to display
  QString current_msg_;
  
  // Subscriber to input messages topic
  ros::Subscriber message_subscriber_;
  
  // Subscriber to input messages topic
  ros::Publisher command_publisher_;
  
  // The ROS node handle.
  ros::NodeHandle nh_;
};

} // team_nust_visualizer_plugins

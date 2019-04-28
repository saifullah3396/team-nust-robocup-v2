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

#include <vector>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <rviz/panel.h>

class QComboBox;
class QPushButton;
class QSlider;
class QLineEdit;
class QTableWidget;
class QTableWidgetItem;
class QVBoxLayout;
class QStackedWidget;

#define MAX_TEXT_BUFFER 100

namespace team_nust_visualizer_plugins
{

class RequestsHandler: public rviz::Panel
{
Q_OBJECT
public:
  RequestsHandler(QWidget* parent = 0);
  ~RequestsHandler() {}

public Q_SLOTS:
  void updateModuleLayout();
  void updateRequestLayout();

private Q_SLOTS:
  void publishSettings();

private:
  //! Drop down menu for module types
  QComboBox* modules_combo_;

  //! Current JSON for request that is under consideration
  Json::Value currentRequest;

  //! Drop down menu for request types
  std::vector<QComboBox*> requests_combo_;

  //! Module layouts
  QStackedWidget* modules_stack_;

  //! Requests layouts for each module
  std::vector<QStackedWidget*> request_widgets_stack_;

  // Main vertical layout
  QVBoxLayout* v_layout_;

  // Publishes the output to robot as a user command
  ros::Publisher requests_publisher_;

  // The ROS node handle.
  ros::NodeHandle nh_;
};

} // team_nust_visualizer_plugins

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

class QPushButton;
class QComboBox;
class QSlider;
class QLineEdit;
class QTableWidget;
class QTableWidgetItem;
template <typename T>
class Camera;

#define MAX_TEXT_BUFFER 100

namespace team_nust_visualizer_plugins
{
  
class CameraCalibration: public rviz::Panel
{
Q_OBJECT
public:
  CameraCalibration(QWidget* parent = 0);
  ~CameraCalibration() {}
  
virtual void load(const rviz::Config& config);
virtual void save(rviz::Config config) const;

public Q_SLOTS:
  void updateConfiguration();
  void updateCamera();
  void saveConfiguration();

private Q_SLOTS:
  void publishSettings();

private: 
  // Changes the camera
  void reset(const QString& camera_name);

  // Pointer to camera class for relevant settings
  Camera<float>* camera;

  // Name of the handled robot necessary to find robot configuration files
  QString robot_name_;

  // Name of the camera
  QString camera_name_;

  // Editor for robot name
  QComboBox* robot_name_combo_box_;

  // Editor for camera name
  QComboBox* camera_name_combo_box_;

  // Qt sliders for all the setting parameters
  std::vector<QSlider*> sliders;

  // Configuration save button
  QPushButton* save_button_;

  // Publishes the output to robot as a user command
  ros::Publisher camera_settings_publisher_;
  
  // The ROS node handle.
  ros::NodeHandle nh_;
};

} // team_nust_visualizer_plugins

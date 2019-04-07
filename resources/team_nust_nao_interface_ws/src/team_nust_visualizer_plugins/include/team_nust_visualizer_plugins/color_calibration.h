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
  
class ColorCalibration: public rviz::Panel
{
Q_OBJECT
public:
  ColorCalibration(QWidget* parent = 0);
  ~ColorCalibration() {}
  
virtual void load(const rviz::Config& config);
virtual void save(rviz::Config config) const;

public Q_SLOTS:
  void updateConfiguration();
  void updateCamera();
  void updateLayout();
  void updateTableLayout();
  void saveConfiguration();

private Q_SLOTS:
  void publishSettings();

private:     
  void readConfiguration();

  // Changes the camera
  void reset(const QString& camera_name);

  //! Drop down menu for colors
  QComboBox* combo_box_;

  //! Drop down menu for color tables
  std::vector<QComboBox*> combo_box_color_tables_;

  //! Color layouts
  QStackedWidget* colors_stack_;

  //! Slider layouts
  std::vector<QStackedWidget*> slider_widgets_stack;

  // Main vertical layout
  QVBoxLayout* v_layout;

  // Name of the handled robot necessary to find robot configuration files
  QString robot_name_;

  // Name of the camera
  QString camera_name_;

  // Editor for robot name
  QComboBox* robot_name_combo_box_;

  // Editor for camera name
  QComboBox* camera_name_combo_box_;

  // Configuration save button
  QPushButton* save_button_;

  // Qt sliders for all the setting parameters
  std::vector<std::vector<std::vector<QSlider*> > > sliders;

  // Publishes the output to robot as a user command
  ros::Publisher camera_settings_publisher_;
  
  // The ROS node handle.
  ros::NodeHandle nh_;
};

} // team_nust_visualizer_plugins

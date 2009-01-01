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
#include <QPainter>
#include <QLabel>
#include <QVBoxLayout>
#include <QLineEdit>
#include <QPushButton>
#include <QHBoxLayout>
#include <QComboBox>
#include <QSlider>
#include <QWidget>
#include <QScrollArea>
#include <json/json.h>
#include <std_msgs/String.h>
#include "camera_calibration.h"
#include "constants.h"
#include "Utils/include/DataHolders/Camera.h"
#include "Utils/include/HardwareIds.h"
#include "Utils/include/ConfigMacros.h"
#include "Utils/include/PrintUtils.h"

namespace team_nust_visualizer_plugins
{

vector<string> slider_names
{
  "torsoRollOffset",
  "torsoPitchOffset"
};

TorsoCalibration::TorsoCalibration(QWidget* parent) : rviz::Panel(parent)
{
  ConfigManager::setDirPaths("Robots/Sim/");
  torso_settings_publisher_ = nh_.advertise<std_msgs::String>("/team_nust_user_cmds", 1000);
  QVBoxLayout* v_layout = new QVBoxLayout();
  QWidget* scroll_content = new QWidget;
  scroll_content->setLayout(v_layout);
  QScrollArea* scroll_area = new QScrollArea;
  scroll_area->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  scroll_area->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  scroll_area->setWidgetResizable(true);
  scroll_area->setWidget(scroll_content);
  vector<int> torso_settings;
  torso_settings.resize(2);
  sliders.resize(2);
  QHBoxLayout* text_layout_robot_name = new QHBoxLayout();
  text_layout_robot_name->addWidget(new QLabel("Robot Name:"));
  robot_name_combo_box_ = new QComboBox();
    for (size_t i = 0; i < NUM_ROBOT_NAMES; ++i) {
    robot_name_combo_box_->addItem(QString::fromStdString(robot_names[i]));
  }
  text_layout_robot_name->addWidget(robot_name_combo_box_);
  v_layout->addLayout(text_layout_robot_name);

  for (size_t i = 0; i < slider_names.size(); ++i) {
    sliders[i] = new QSlider(Qt::Horizontal, this);
    QLabel* label = new QLabel();
    label->setText(QString::fromStdString(slider_names[i]));
    v_layout->addWidget(label);
    sliders[i]->setRange(100, 100);
    sliders[i]->setValue(0);
    connect(sliders[i], SIGNAL(sliderReleased()), this, SLOT(publishSettings()));
    v_layout->addWidget(sliders[i]);
  }
  save_button_ = new QPushButton("Save", this);
  v_layout->addWidget(save_button_);  // Configuration save button
  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->addWidget(scroll_area);

  // Next we make signal/slot connections.
  connect(robot_name_combo_box_, SIGNAL(currentIndexChanged(int)), this, SLOT(updateConfiguration()));
  connect(save_button_, SIGNAL (released()), this, SLOT (saveConfiguration()));

  setLayout(main_layout);
}

void TorsoCalibration::updateConfiguration()
{
  auto robot_name = robot_names[robot_name_combo_box_->currentIndex()];
  ConfigManager::setDirPaths("Robots/" + robot_name + "/");
  reset(camera_name_);
}

void TorsoCalibration::publishSettings()
{
  Json::Value root;
  Json::Value json_settings;
  json_settings.append(camera->name == "visionTop" ? toUType(CameraId::headTop) : toUType(CameraId::headBottom));
  for (size_t i = 0; i < slider_names.size(); ++i) {
    json_settings.append(sliders[i]->value());
  }
  root["CameraModule"]["settingParams"] = json_settings;
  Json::FastWriter fastWriter;
  std::string output = fastWriter.write(root);
  std_msgs::String msg;
  msg.data = output.c_str();
  torso_settings_publisher_.publish(msg);
}

void TorsoCalibration::saveConfiguration()
{
  try {
    using namespace std;
    string jsonConfigPath;
    jsonConfigPath = ConfigManager::getConfigDirPath() + "KinematicSettings.json";
    ifstream config(jsonConfigPath, fstream::binary);
    Json::Value root;
    config >> root;
    for (size_t i = 0; i < slider_names.size(); ++i) {
      root["torsoRotation"][slider_names[i]] = sliders[i]->value() * 0.1;
    }
    std::ofstream out;
    out.open(jsonConfigPath);
    Json::StyledWriter styledWriter;
    out << styledWriter.write(root);
    out.close();
  } catch (Json::Exception& e) {
    LOG_EXCEPTION(e.what());
  }
}

} // end namespace team_nust_visualizer_plugins

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(team_nust_visualizer_plugins::TorsoCalibration, rviz::Panel)

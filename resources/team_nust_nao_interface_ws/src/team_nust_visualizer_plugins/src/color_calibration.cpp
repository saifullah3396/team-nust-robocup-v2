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
#include <QComboBox>
#include <QPushButton>
#include <QHBoxLayout>
#include <QSlider>
#include <QWidget>
#include <QStackedWidget>
#include <QScrollArea>
#include <json/json.h>
#include <std_msgs/String.h>
#include "color_calibration.h"
#include "constants.h"
#include "Utils/include/JsonUtils.h"
#include "Utils/include/HardwareIds.h"
#include "Utils/include/ConfigMacros.h"
#include "Utils/include/PrintUtils.h"
#include "Utils/include/TNColors.h"

namespace team_nust_visualizer_plugins
{

ColorCalibration::ColorCalibration(QWidget* parent) : rviz::Panel(parent)
{
  ConfigManager::setDirPaths("Robots/Sim/");
  camera_settings_publisher_ = nh_.advertise<std_msgs::String>("/team_nust_user_cmds", 1000);
  QVBoxLayout* v_layout = new QVBoxLayout();
  QWidget* scroll_content = new QWidget;
  scroll_content->setLayout(v_layout);
  QScrollArea* scroll_area = new QScrollArea;
  scroll_area->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  scroll_area->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  scroll_area->setWidgetResizable(true);
  scroll_area->setWidget(scroll_content);

  QHBoxLayout* text_layout_robot_name = new QHBoxLayout();
  text_layout_robot_name->addWidget(new QLabel("Robot Name:"));
  robot_name_combo_box_ = new QComboBox();
    for (size_t i = 0; i < NUM_ROBOT_NAMES; ++i) {
    robot_name_combo_box_->addItem(QString::fromStdString(robot_names[i]));
  }
  text_layout_robot_name->addWidget(robot_name_combo_box_);

  QHBoxLayout* text_layout_camera_name = new QHBoxLayout();
  text_layout_camera_name->addWidget(new QLabel("Camera Name:"));
  camera_name_combo_box_ = new QComboBox();
  for (size_t i = 0; i < toUType(CameraId::count); ++i) {
    camera_name_combo_box_->addItem(QString::fromStdString(camera_names[i]));
  }
  text_layout_camera_name->addWidget(camera_name_combo_box_);

  v_layout->addLayout(text_layout_robot_name);
  v_layout->addLayout(text_layout_camera_name);

  combo_box_ = new QComboBox();
  for (size_t i = 0; i < toUType(TNColors::count); ++i) {
    combo_box_->addItem(QString::fromStdString(tn_colors[i]));
  }
  v_layout->addWidget(combo_box_);

  sliders.resize(toUType(TNColors::count));
  combo_box_color_tables_.resize(toUType(TNColors::count));
  slider_widgets_stack.resize(toUType(TNColors::count));
  colors_stack_ = new QStackedWidget();

  auto root = JsonUtils::readJson(ConfigManager::getConfigDirPath() + "ColorConfig.json");
  for (size_t i = 0; i < toUType(TNColors::count); ++i) {
    sliders[i].resize(n_color_tables[i]);
    combo_box_color_tables_[i] = new QComboBox();
    QVBoxLayout* color_layout = new QVBoxLayout();
    color_layout->addWidget(combo_box_color_tables_[i]);
    QWidget* color_layout_widget = new QWidget();
    color_layout_widget->setLayout(color_layout);
    colors_stack_->addWidget(color_layout_widget);
    slider_widgets_stack[i] = new QStackedWidget();
    for (size_t j = 0; j < root[colorNames[i]]["tables"].asInt(); ++j) {
      sliders[i][j].resize(6);
      combo_box_color_tables_[i]->addItem(QString("Table ") + QString::number(j+1));
      QVBoxLayout* slider_layout = new QVBoxLayout();
      for (size_t k = 0; k < 6; ++k) {
        sliders[i][j][k] = new QSlider(Qt::Horizontal);
        QLabel* label = new QLabel("");
        label->setText(QString::fromStdString(slider_names[k]));
        slider_layout->addWidget(label);
        sliders[i][j][k]->setRange(0, 255);
        slider_layout->addWidget(sliders[i][j][k]);
        connect(sliders[i][j][k], SIGNAL(sliderReleased()), this, SLOT(publishSettings()));
      }
      QWidget* layout_widget = new QWidget();
      layout_widget->setLayout(slider_layout);
      slider_widgets_stack[i]->addWidget(layout_widget);
    }
    color_layout->addWidget(slider_widgets_stack[i]);
  }
  v_layout->addWidget(colors_stack_);

  save_button_ = new QPushButton("Save", this);
  v_layout->addWidget(save_button_);  // Configuration save button
  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->addWidget(scroll_area);

  // Next we make signal/slot connections.
  connect(camera_name_combo_box_, SIGNAL(currentIndexChanged(int)), this, SLOT(updateCamera()));
  connect(robot_name_combo_box_, SIGNAL(currentIndexChanged(int)), this, SLOT(updateConfiguration()));
  connect(combo_box_, SIGNAL(currentIndexChanged(int)), this, SLOT(updateLayout()));
  for (size_t i = 0; i < toUType(TNColors::count); ++i)
    connect(combo_box_color_tables_[i], SIGNAL(currentIndexChanged(int)), this, SLOT(updateTableLayout()));
  connect(save_button_, SIGNAL (released()), this, SLOT (saveConfiguration()));
  readConfiguration();
  setLayout(main_layout);
}

void ColorCalibration::updateLayout()
{
  colors_stack_->setCurrentIndex(combo_box_->currentIndex());
}

void ColorCalibration::updateTableLayout()
{
  auto color_index = combo_box_->currentIndex();
  slider_widgets_stack[color_index]->setCurrentIndex(combo_box_color_tables_[color_index]->currentIndex());
}

void ColorCalibration::reset(const QString& camera_name)
{
  Json::Value root;
  camera_name_ = camera_name;
  root["VisionModule"]["debugImageIndex"] =
    camera_name_ == "visionTop" ? toUType(CameraId::headTop) : toUType(CameraId::headBottom);
  Json::FastWriter fastWriter;
  std::string output = fastWriter.write(root);
  std_msgs::String msg;
  msg.data = output.c_str();
  camera_settings_publisher_.publish(msg);
}

void ColorCalibration::updateCamera()
{
  reset(QString::fromStdString(camera_names[camera_name_combo_box_->currentIndex()]));
  Q_EMIT configChanged();
}

void ColorCalibration::publishSettings()
{
  Json::Value root;
  root["ColorHandler"]["colorIndex"] = combo_box_->currentIndex();
  root["ColorHandler"]["tableIndex"] = combo_box_color_tables_[combo_box_->currentIndex()]->currentIndex();
  auto key = "lower";
  for (int i = 0; i < 6; ++i) {
    if (i >= 3)
      key = "upper";
    root["ColorHandler"][key].append(
      sliders[combo_box_->currentIndex()][combo_box_color_tables_[combo_box_->currentIndex()]->currentIndex()][i]->value());
  }
  root["VisionModule"]["sendBinaryImage"] = combo_box_->currentIndex();
  Json::FastWriter fastWriter;
  std::string output = fastWriter.write(root);
  std_msgs::String msg;
  msg.data = output.c_str();
  camera_settings_publisher_.publish(msg);
}

void ColorCalibration::updateConfiguration()
{
  auto robot_name = robot_names[robot_name_combo_box_->currentIndex()];
  ConfigManager::setDirPaths("Robots/" + robot_name + "/");
  readConfiguration();
  reset(camera_name_);
}

void ColorCalibration::saveConfiguration()
{
  try {
    using namespace std;
    string jsonConfigPath;
    jsonConfigPath = ConfigManager::getConfigDirPath() + "ColorConfig.json";
    ifstream config(jsonConfigPath, fstream::binary);
    Json::Value root;
    config >> root;
    for (int i = 0; i < toUType(TNColors::count); ++i) {
      auto lower = root[tn_colors[i]]["lower"];
      for (int k = 0; k < lower.size(); ++k) {
        for (int j = 0; j < 3; ++j) {
          root[tn_colors[i]]["lower"][k][j] = sliders[i][k][j]->value();
        }
      }
      for (int k = 0; k < lower.size(); ++k) {
        for (int j = 0; j < 3; ++j) {
          root[tn_colors[i]]["upper"][k][j] = sliders[i][k][j+3]->value();
        }
      }
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

void ColorCalibration::readConfiguration()
{
  try {
    using namespace std;
    string jsonConfigPath;
    jsonConfigPath = ConfigManager::getConfigDirPath() + "ColorConfig.json";
    ifstream config(jsonConfigPath, fstream::binary);
    Json::Value root;
    config >> root;
    for (int i = 0; i < toUType(TNColors::count); ++i) {
      auto lower = root[tn_colors[i]]["lower"];
      for (int k = 0; k < lower.size(); ++k) {
        for (int j = 0; j < 3; ++j) {
          sliders[i][k][j]->setValue(root[tn_colors[i]]["lower"][k][j].asInt());
        }
      }
      for (int k = 0; k < lower.size(); ++k) {
        for (int j = 0; j < 3; ++j) {
          sliders[i][k][j+3]->setValue(root[tn_colors[i]]["upper"][k][j].asInt());
        }
      }
    }
  } catch (Json::Exception& e) {
    ROS_ERROR("%s", e.what());
  }
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void ColorCalibration::save(rviz::Config config) const
{
  rviz::Panel::save(config);
  config.mapSetValue("Camera Name:", camera_name_);
}

// Load all configuration data for this panel from the given Config object.
void ColorCalibration::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
  reset(QString::fromStdString(camera_names[camera_name_combo_box_->currentIndex()]));
}

} // end namespace team_nust_visualizer_plugins

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(team_nust_visualizer_plugins::ColorCalibration, rviz::Panel)

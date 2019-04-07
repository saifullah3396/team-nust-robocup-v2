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
#include <QLineEdit>
#include <QVBoxLayout>
#include <QPlainTextEdit>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QHeaderView>

#include "sensors_info.h"
#include "Utils/include/HardwareIds.h"

using namespace std;

#define DEFINE_SENSOR_CALLBACK_FUNC(Name, Type, Index) \
  void SensorsInfo::Name(const team_nust_msgs::SensorState::ConstPtr& state) \
  { \
    static bool names_added = false; \
    auto table = data_tables[toUType(SensorTypes::Index) - toUType(JointSensorTypes::count) + 1]; \
    if (!names_added) { \
      for (size_t i = 0; i < toUType(Type::count); ++i) { \
        table->item(i, 0)->setText(QString::fromStdString(state->name[i])); \
        table->item(i, 1)->setText(QString::number(state->value[i])); \
      } \
    } else { \
      for (size_t i = 0; i < toUType(Type::count); ++i) { \
        data_tables[toUType(SensorTypes::Index) - toUType(JointSensorTypes::count) + 1]-> \
          item(i, 1)->setText(QString::number(state->value[i])); \
      } \
    } \
    names_added = true; \
  }

namespace team_nust_visualizer_plugins
{
  std::string sensorNames[toUType(SensorTypes::count) - toUType(JointSensorTypes::count) + 1] 
  {
    "Joint Sensors",
    "Hand Sensors",
    "Touch Sensors",
    "Switch Sensors",
    "Battery Sensors",
    "Inertial Sensors",
    "Sonar Sensors",
    "Fsr Sensors",
    "Led Sensors"
  };
  
  std::string sensor_subscriber_topics[toUType(SensorTypes::count) - toUType(JointSensorTypes::count)] 
  {
    "/nao_hand_sensors",
    "/nao_touch_sensors",
    "/nao_switch_sensors",
    "/nao_battery_sensors",
    "/nao_inertial_sensors",
    "/nao_sonar_sensors",
    "/nao_fsr_sensors",
    "/nao_led_sensors"
  };

SensorsInfo::SensorsInfo(QWidget* parent) : rviz::Panel(parent)
{
  //switch_sensors_subscriber = nh_.subscribe("/nao_switch_sensors", 5, &SensorsInfo::updateSwitchSensors, this);
  QHBoxLayout* layout = new QHBoxLayout();
  auto n_js_types = toUType(JointSensorTypes::count);
  n_tables = toUType(SensorTypes::count) - n_js_types + 1;
  data_tables.resize(n_tables);
  QStringList labels;
  labels << "Sensor" << "Value";
  for (size_t i = 0; i < n_tables; ++i) {
    data_tables[i] = new QTableWidget();
    data_tables[i]->setEditTriggers(QAbstractItemView::NoEditTriggers);
    data_tables[i]->setColumnCount(2);
    data_tables[i]->setHorizontalHeaderLabels(labels);
    data_tables[i]->setShowGrid(false);
    data_tables[i]->horizontalHeader()->setResizeMode(QHeaderView::ResizeToContents); 
    data_tables[i]->verticalHeader()->setResizeMode(QHeaderView::ResizeToContents); 
//  data_tables[i]->verticalHeader()->setVisible(false);
  }
  QTabWidget *tabs = new QTabWidget();
  QStringList js_labels;
  js_labels << "Joint" << "Position" << "Stiffness" << "Temperature" << "Current";
  data_tables[toUType(SensorTypes::joints)]->setRowCount(toUType(Joints::count));
  data_tables[toUType(SensorTypes::joints)]->setColumnCount(n_js_types + 1);
  data_tables[toUType(SensorTypes::joints)]->setHorizontalHeaderLabels(js_labels);
  data_tables[toUType(SensorTypes::handSensors) - n_js_types + 1]->setRowCount(toUType(RobotHands::count));
  data_tables[toUType(SensorTypes::touchSensors) - n_js_types + 1]->setRowCount(toUType(TouchSensors::count));
  data_tables[toUType(SensorTypes::switchSensors) - n_js_types + 1]->setRowCount(toUType(SwitchSensors::count));
  data_tables[toUType(SensorTypes::batterySensors) - n_js_types + 1]->setRowCount(toUType(BatterySensors::count));
  data_tables[toUType(SensorTypes::inertialSensors) - n_js_types + 1]->setRowCount(toUType(InertialSensors::count));
  data_tables[toUType(SensorTypes::sonarSensors) - n_js_types + 1]->setRowCount(toUType(SonarSensors::count));
  data_tables[toUType(SensorTypes::fsrSensors) - n_js_types + 1]->setRowCount(toUType(FsrSensors::count));
  data_tables[toUType(SensorTypes::ledSensors) - n_js_types + 1]->setRowCount(toUType(LedActuators::count));
  for (size_t i = 0; i < n_tables; ++i) {
    for (size_t j = 0; j < data_tables[i]->rowCount(); ++j) {
      for (size_t k = 0; k < data_tables[i]->columnCount(); ++k) {
        QTableWidgetItem* item = new QTableWidgetItem("");
        item->setFlags(item->flags() ^ Qt::ItemIsEditable);
        data_tables[i]->setItem(j, k, item);
      }
    }
    tabs->addTab(data_tables[i], QString::fromStdString(sensorNames[i]));
  }
  layout->addWidget(tabs);
  
  joint_states_subscriber = nh_.subscribe("/joint_states", 5, &SensorsInfo::updateJointStates, this);
  joints_info_subscriber = nh_.subscribe("/joint_info", 5, &SensorsInfo::updateJointsInfo, this);
  sensor_subscribers.resize(toUType(SensorTypes::count) - n_js_types);
  sensor_subscribers[toUType(SensorTypes::handSensors) - n_js_types] = 
    nh_.subscribe(
      sensor_subscriber_topics[toUType(SensorTypes::handSensors) - n_js_types], 
      5, 
      &SensorsInfo::updateHandSensors, 
      this);
  sensor_subscribers[toUType(SensorTypes::touchSensors) - n_js_types] = 
    nh_.subscribe(
      sensor_subscriber_topics[toUType(SensorTypes::touchSensors) - n_js_types], 
      5, 
      &SensorsInfo::updateTouchSensors, 
      this);    
  sensor_subscribers[toUType(SensorTypes::switchSensors) - n_js_types] = 
    nh_.subscribe(
      sensor_subscriber_topics[toUType(SensorTypes::switchSensors) - n_js_types], 
      5, 
      &SensorsInfo::updateSwitchSensors, 
      this); 
  sensor_subscribers[toUType(SensorTypes::batterySensors) - n_js_types] = 
    nh_.subscribe(
      sensor_subscriber_topics[toUType(SensorTypes::batterySensors) - n_js_types], 
      5, 
      &SensorsInfo::updateBatterySensors, 
      this); 
  sensor_subscribers[toUType(SensorTypes::inertialSensors) - n_js_types] = 
    nh_.subscribe(
      sensor_subscriber_topics[toUType(SensorTypes::inertialSensors) - n_js_types], 
      5, 
      &SensorsInfo::updateInertialSensors, 
      this); 
  sensor_subscribers[toUType(SensorTypes::sonarSensors) - n_js_types] = 
    nh_.subscribe(
      sensor_subscriber_topics[toUType(SensorTypes::sonarSensors) - n_js_types], 
      5, 
      &SensorsInfo::updateSonarSensors, 
      this); 
  sensor_subscribers[toUType(SensorTypes::fsrSensors) - n_js_types] = 
    nh_.subscribe(
      sensor_subscriber_topics[toUType(SensorTypes::fsrSensors) - n_js_types], 
      5, 
      &SensorsInfo::updateFsrSensors, 
      this); 
  sensor_subscribers[toUType(SensorTypes::ledSensors) - n_js_types] = 
    nh_.subscribe(
      sensor_subscriber_topics[toUType(SensorTypes::ledSensors) - n_js_types], 
      5, 
      &SensorsInfo::updateLedSensors, 
      this); 
  setLayout(layout);
}

void SensorsInfo::updateJointStates(const sensor_msgs::JointState::ConstPtr& state)
{
  for (size_t i = 0; i < toUType(Joints::count); ++i) {
    data_tables[toUType(SensorTypes::joints)]->
      item(i, 1)->
        setText(QString::number(state->position[i] * 57.324840764));
  }
}

void SensorsInfo::updateJointsInfo(const team_nust_msgs::JointInfo::ConstPtr& info)
{
  static bool names_added = false;
  if (!names_added) {
    for (size_t i = 0; i < toUType(Joints::count); ++i) {
      data_tables[toUType(SensorTypes::joints)]->item(i, 0)->setText(QString(info->name[i].c_str()));
      data_tables[toUType(SensorTypes::joints)]->item(i, 2)->setText(QString::number(info->stiffness[i]));
      data_tables[toUType(SensorTypes::joints)]->item(i, 3)->setText(QString::number(info->temperature[i]));
      data_tables[toUType(SensorTypes::joints)]->item(i, 4)->setText(QString::number(info->current[i]));
    }
  } else {
    for (size_t i = 0; i < toUType(Joints::count); ++i) {
      data_tables[toUType(SensorTypes::joints)]->item(i, 2)->setText(QString::number(info->stiffness[i]));
      data_tables[toUType(SensorTypes::joints)]->item(i, 3)->setText(QString::number(info->temperature[i]));
      data_tables[toUType(SensorTypes::joints)]->item(i, 4)->setText(QString::number(info->current[i]));
    }
  }
  names_added = true;
}

DEFINE_SENSOR_CALLBACK_FUNC(updateHandSensors, RobotHands, handSensors);
DEFINE_SENSOR_CALLBACK_FUNC(updateTouchSensors, TouchSensors, touchSensors);
DEFINE_SENSOR_CALLBACK_FUNC(updateSwitchSensors, SwitchSensors, switchSensors);
DEFINE_SENSOR_CALLBACK_FUNC(updateBatterySensors, BatterySensors, batterySensors);
DEFINE_SENSOR_CALLBACK_FUNC(updateInertialSensors, InertialSensors, inertialSensors);
DEFINE_SENSOR_CALLBACK_FUNC(updateSonarSensors, SonarSensors, sonarSensors);
DEFINE_SENSOR_CALLBACK_FUNC(updateFsrSensors, FsrSensors, fsrSensors);
DEFINE_SENSOR_CALLBACK_FUNC(updateLedSensors, LedActuators, ledSensors);

} // end namespace team_nust_visualizer_plugins

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(team_nust_visualizer_plugins::SensorsInfo, rviz::Panel)

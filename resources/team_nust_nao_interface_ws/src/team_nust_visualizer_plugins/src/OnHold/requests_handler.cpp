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
#include "TeamNUSTSPL/include/TNSPLModuleIds.h"
#include "PlanningModule/include/PlanningRequest.h"
#include "MotionModule/include/MotionRequest.h"
#include "GBModule/include/GBRequest.h"
#include "VisionModule/include/VisionRequest.h"
#include "LocalizationModule/include/LocalizationRequest.h"
#include "GameCommModule/include/GameCommRequest.h"
#include "UserCommModule/include/UserCommRequest.h"
#include "ControlModule/include/LolaRequest.h"

namespace team_nust_visualizer_plugins
{

string module_names[toUType(TNSPLModules::count)] {
  "planning",
  "motion",
  "gb",
  "vision",
  "localization",
  "gameComm",
  "userComm",
  "lola"
};

const unsigned n_requests_in_module[toUType(TNSPLModules::count)] {
  toUType(PlanningRequestIds::count),
  toUType(MotionRequestIds::count),
  toUType(GBRequestIds::count),
  toUType(VisionRequestIds::count),
  toUType(LocalizationRequestIds::count),
  toUType(GameCommRequestIds::count),
  toUType(UserCommRequestIds::count),
  toUType(LolaRequestIds::count)
};

RequestsHandler::RequestsHandler(QWidget* parent) : rviz::Panel(parent)
{
  requests_publisher_ = nh_.advertise<std_msgs::String>("/team_nust_user_cmds", 1000);
  QVBoxLayout* v_layout = new QVBoxLayout();
  QWidget* scroll_content = new QWidget;
  scroll_content->setLayout(v_layout);
  QScrollArea* scroll_area = new QScrollArea;
  scroll_area->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  scroll_area->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  scroll_area->setWidgetResizable(true);
  scroll_area->setWidget(scroll_content);

  modules_combo_ = new QComboBox();
  for (size_t i = 0; i < toUType(TNSPLModules::count); ++i) {
    modules_combo_->addItem(QString::fromStdString(module_names[i]));
  }
  v_layout->addWidget(modules_combo_);

  requests_combo_.resize(toUType(TNSPLModules::count));
  modules_stack_ = new QStackedWidget();
  for (size_t i = 0; i < toUType(TNSPLModules::count); ++i) {
    requests_combo_[i] = new QComboBox();
    QVBoxLayout* module_layout = new QVBoxLayout();
    module_layout->addWidget(requests_combo_[i]);
    QWidget* module_layout_widget = new QWidget();
    module_layout_widget->setLayout(module_layout);
    module_stack_->addWidget(module_layout_widget);
    request_widgets_stack[i] = new QStackedWidget();
    for (size_t j = 0; j < n_requests_in_module[i]; ++j) {
      requests_combo_[i]->addItem(QString("Request ") + QString::number(j+1));
      QVBoxLayout* request_layout = new QVBoxLayout();
      QWidget* layout_widget = new QWidget();
      layout_widget->setLayout(request_layout);
      request_widgets_stack[i]->addWidget(layout_widget);
    }
    module_layout->addWidget(request_widgets_stack[i]);
  }
  v_layout->addWidget(module_stack_);

  send_button_ = new QPushButton("Send", this);
  v_layout->addWidget(send_button_);  // Configuration save button
  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->addWidget(scroll_area);

  // Next we make signal/slot connections.
  connect(modules_combo_, SIGNAL(currentIndexChanged(int)), this, SLOT(updateModuleLayout()));
  for (size_t i = 0; i < toUType(TNSPLModules::count); ++i)
    connect(requests_combo_[i], SIGNAL(currentIndexChanged(int)), this, SLOT(updateRequestLayout()));
  connect(save_button_, SIGNAL (released()), this, SLOT (publishSettings()));
  setLayout(main_layout);
}

void RequestsHandler::updateModuleLayout()
{
  modules_stack_->setCurrentIndex(modules_combo_->currentIndex());
}

void RequestsHandler::updateRequestLayout()
{
  auto color_index = modules_combo_->currentIndex();
  request_widgets_stack[color_index]->setCurrentIndex(requests_combo_[color_index]->currentIndex());
}

void RequestsHandler::publishSettings()
{
  Json::Value root;
  root["moduleId"] = modules_combo_->currentIndex();
  root["requestId"] = requests_combo_[modules_combo_->currentIndex()]->currentIndex();
  root.append(currentRequest);
  Json::FastWriter fastWriter;
  std::string output = fastWriter.write(root);
  std_msgs::String msg;
  msg.data = output.c_str();
  requests_publisher_.publish(msg);
}

} // end namespace team_nust_visualizer_plugins

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(team_nust_visualizer_plugins::RequestsHandler, rviz::Panel)

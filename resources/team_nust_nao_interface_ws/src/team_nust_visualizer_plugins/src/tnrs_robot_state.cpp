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

#include <std_msgs/String.h>
#include "tnrs_robot_state.h"
#include "TeamNUSTSPL/include/TNSPLModuleIds.h"
#include "Utils/include/EnumUtils.h"
#include "Utils/include/JsonUtils.h"

using namespace std;

namespace team_nust_visualizer_plugins
{

enum class StateTables : unsigned int {
  modulesState,
  robotState,
  behaviorState,
  gameState,
  localizerState,
  count,
};

string state_table_names[toUType(StateTables::count)] {
  "Modules State",
  "Robot State",
  "Behavior State",
  "Game State",
  "Localizer State"
};

enum class ModulesStateItems : unsigned int {
  motionThreadPeriod,
  planningThreadPeriod,
  gbThreadPeriod,
  visionThreadPeriod,
  localizerThreadPeriod,
  userCommThreadPeriod,
  gameCommThreadPeriod,
  motionThreadTimeTaken,
  planningThreadTimeTaken,
  gbThreadTimeTaken,
  visionThreadTimeTaken,
  localizerThreadTimeTaken,
  userCommThreadTimeTaken,
  gameCommThreadTimeTaken,
  count
};

string modules_state_items_names[toUType(ModulesStateItems::count)] {
  "motionThreadPeriod",
  "planningThreadPeriod",
  "gbThreadPeriod",
  "visionThreadPeriod",
  "localizationThreadPeriod",
  "userCommThreadPeriod",
  "gameCommThreadPeriod",
  "motionThreadTimeTaken",
  "planningThreadTimeTaken",
  "gbThreadTimeTaken",
  "visionThreadTimeTaken",
  "localizationThreadTimeTaken",
  "userCommThreadTimeTaken",
  "gameCommThreadTimeTaken"
};

enum class RobotStateItems : unsigned int {
  stiffnessState,
  postureState,
  robotFallen,
  robotInMotion,
  footOnGround,
  count
};

string robot_state_items_names[toUType(RobotStateItems::count)] {
  "stiffnessState",
  "postureState",
  "robotFallen",
  "robotInMotion",
  "footOnGround"
};

enum class GameStateItems : unsigned int {
  playerNumber,
  teamNumber,
  teamPort,
  teamColor,
  robocupRole,
  robotIntention,
  whistleDetected,
  count
};

string game_state_items_names[toUType(GameStateItems::count)] {
  "playerNumber",
  "teamNumber",
  "teamPort",
  "teamColor",
  "robocupRole",
  "robotIntention",
  "whistleDetected"
};

enum class LocalizerStateItems : unsigned int {
  robotLocalized,
  positionConfidence,
  sideConfidence,
  robotOnSideLine,
  localizeWithLastKnown,
  landmarksFound,
  count
};

string localizer_state_items_names[toUType(LocalizerStateItems::count)] {
  "robotLocalized",
  "positionConfidence",
  "sideConfidence",
  "robotOnSideLine",
  "localizeWithLastKnown",
  "landmarksFound"
};

enum class BehaviorStateItems : unsigned int {
  planningBehavior,
  gbBehavior,
  motionBehavior,
  count
};

string behavior_state_items_names[toUType(BehaviorStateItems::count)] {
  "planningBehavior",
  "gbBehavior",
  "motionBehavior"
};

#define SET_TABLE_NAMES(TableIndex, TableEnum, TableNames) \
  for (size_t i = 0; i < toUType(TableEnum::count); ++i) { \
    data_tables[toUType(StateTables::TableIndex)]->item(i, 0)-> \
      setText(QString::fromStdString(TableNames[i])); \
  }

#define SET_ITEM_VALUE(TableIndex, ItemIndex, Value) \
  data_tables[toUType(TableIndex)]->item(toUType(ItemIndex), 1)->setText(Value);

#define SET_ITEM_WIDGET(TableIndex, ItemIndex, Widget) \
  data_tables[toUType(TableIndex)]->setCellWidget(toUType(ItemIndex), 1, Widget);

#define GET_ITEM_WIDGET(TableIndex, ItemIndex) \
  data_tables[toUType(TableIndex)]->cellWidget(toUType(ItemIndex), 1)

#define BOOL_TO_STRING(OUT, IN) \
  if (IN) OUT = "True"; \
  else OUT = "False";

RobotState::RobotState(QWidget* parent) : rviz::Panel(parent)
{
  connectionStatus = new QLabel();
  connectionStatus->setText("Disconnected.");
  data_tables.resize(toUType(StateTables::count));
  QStringList labels;
  labels << "Data" << "Value";
  for (size_t i = 0; i < data_tables.size(); ++i) {
    data_tables[i] = new QTableWidget();
    data_tables[i]->setEditTriggers(QAbstractItemView::NoEditTriggers);
    data_tables[i]->setColumnCount(2);
    data_tables[i]->setHorizontalHeaderLabels(labels);
    data_tables[i]->setShowGrid(false);
    data_tables[i]->horizontalHeader()->setResizeMode(QHeaderView::ResizeToContents);
    data_tables[i]->verticalHeader()->setResizeMode(QHeaderView::ResizeToContents);
  }
  data_tables[toUType(StateTables::modulesState)]->setRowCount(toUType(ModulesStateItems::count));
  data_tables[toUType(StateTables::robotState)]->setRowCount(toUType(RobotStateItems::count));
  data_tables[toUType(StateTables::gameState)]->setRowCount(toUType(GameStateItems::count));
  data_tables[toUType(StateTables::localizerState)]->setRowCount(toUType(LocalizerStateItems::count));
  data_tables[toUType(StateTables::behaviorState)]->setRowCount(toUType(BehaviorStateItems::count));
  QVBoxLayout* layout = new QVBoxLayout();
  QTabWidget *tabs = new QTabWidget();
  for (size_t i = 0; i < data_tables.size(); ++i) {
    for (size_t j = 0; j < data_tables[i]->rowCount(); ++j) {
      for (size_t k = 0; k < data_tables[i]->columnCount(); ++k) {
        QTableWidgetItem* item = new QTableWidgetItem("");
        item->setFlags(item->flags() ^ Qt::ItemIsEditable);
        data_tables[i]->setItem(j, k, item);
      }
    }
    tabs->addTab(data_tables[i], QString::fromStdString(state_table_names[i]));
  }
  SET_TABLE_NAMES(modulesState, ModulesStateItems, modules_state_items_names)
  SET_TABLE_NAMES(robotState, RobotStateItems, robot_state_items_names)
  SET_TABLE_NAMES(gameState, GameStateItems, game_state_items_names)
  SET_TABLE_NAMES(localizerState, LocalizerStateItems, localizer_state_items_names)
  SET_TABLE_NAMES(behaviorState, BehaviorStateItems, behavior_state_items_names)

  SET_ITEM_WIDGET(StateTables::behaviorState, BehaviorStateItems::planningBehavior, new QLabel());
  SET_ITEM_WIDGET(StateTables::behaviorState, BehaviorStateItems::motionBehavior, new QLabel());
  SET_ITEM_WIDGET(StateTables::behaviorState, BehaviorStateItems::gbBehavior, new QLabel());

  tnrs_state_subscriber = nh_.subscribe("/team_nust_state", 1, &RobotState::updateTeamNUSTState, this);
  localizer_state_subscriber = nh_.subscribe("/localization_state", 1, &RobotState::updateLocalizerState, this);
  pb_info_subscriber = nh_.subscribe("/pb_info", 1, &RobotState::updatePBInfo, this);
  mb_info_subscriber = nh_.subscribe("/mb_info", 1, &RobotState::updateMBInfo, this);
  gb_info_subscriber = nh_.subscribe("/gb_info", 1, &RobotState::updateGBInfo, this);
  layout->addWidget(tabs);
  layout->addWidget(connectionStatus);
  setLayout(layout);
}

QString RobotState::getBehaviorContent(const team_nust_msgs::BehaviorInfo::ConstPtr& b_info)
{
  if (b_info->name.empty())
    return QString();
  std::string config;
  Json::Value root;
  Json::Reader reader;
  bool parsed = reader.parse(b_info->config, root);
  if (parsed)
  {
    Json::StyledWriter styledWriter;
    config = styledWriter.write(root);
  } else {
    ROS_INFO("Unable to parse configuration associated with %s", b_info->name.c_str());
  }

  QString b_string =
    QString("Name: ") + QString::fromStdString(b_info->name) + QString("\n") +
    QString("State: ") + QString::fromStdString(b_info->fsm_state) + QString("\n") +
    QString("Initiated: ") + QString::number(b_info->initiated) + QString("\n") +
    QString("Running: ") + QString::number(b_info->running) + QString("\n") +
    QString("Paused: ") + QString::number(b_info->paused) + QString("\n") +
    QString("Finished: ") + QString::number(b_info->finished) + QString("\n") +
    QString("Config: ") + QString::fromStdString(config);

  return b_string;
}

void RobotState::updatePBInfo(const team_nust_msgs::BehaviorInfo::ConstPtr& b_info)
{
  static_cast<QLabel*>(
    GET_ITEM_WIDGET(
      StateTables::behaviorState, BehaviorStateItems::planningBehavior))->
        setText(getBehaviorContent(b_info));
}

void RobotState::updateMBInfo(const team_nust_msgs::BehaviorInfo::ConstPtr& b_info)
{
  static_cast<QLabel*>(
    GET_ITEM_WIDGET(
      StateTables::behaviorState, BehaviorStateItems::motionBehavior))->
        setText(getBehaviorContent(b_info));
}

void RobotState::updateGBInfo(const team_nust_msgs::BehaviorInfo::ConstPtr& b_info)
{
  static_cast<QLabel*>(
    GET_ITEM_WIDGET(
      StateTables::behaviorState, BehaviorStateItems::gbBehavior))->
        setText(getBehaviorContent(b_info));
}

void RobotState::updateTeamNUSTState(const team_nust_msgs::TeamNUSTState::ConstPtr& state)
{
  static int prev_beat = 0;
  if (state->heart_beat != prev_beat)
    connectionStatus->setText("Connected to robot.");
  else
    connectionStatus->setText("Disconnected.");
  prev_beat = state->heart_beat;
  setModulesState(state);
  setRobotState(state);
  setGameState(state);
}

void RobotState::setModulesState(const team_nust_msgs::TeamNUSTState::ConstPtr& state)
{
  SET_ITEM_VALUE(StateTables::modulesState, ModulesStateItems::motionThreadPeriod, QString::number(state->motion_thread_period));
  SET_ITEM_VALUE(StateTables::modulesState, ModulesStateItems::planningThreadPeriod, QString::number(state->planning_thread_period));
  SET_ITEM_VALUE(StateTables::modulesState, ModulesStateItems::gbThreadPeriod, QString::number(state->gb_thread_period));
  SET_ITEM_VALUE(StateTables::modulesState, ModulesStateItems::visionThreadPeriod, QString::number(state->vision_thread_period));
  SET_ITEM_VALUE(StateTables::modulesState, ModulesStateItems::localizerThreadPeriod, QString::number(state->localization_thread_period));
  SET_ITEM_VALUE(StateTables::modulesState, ModulesStateItems::userCommThreadPeriod, QString::number(state->user_comm_thread_period));
  SET_ITEM_VALUE(StateTables::modulesState, ModulesStateItems::gameCommThreadPeriod, QString::number(state->game_comm_thread_period));

  SET_ITEM_VALUE(StateTables::modulesState, ModulesStateItems::motionThreadTimeTaken, QString::number(state->motion_time_taken));
  SET_ITEM_VALUE(StateTables::modulesState, ModulesStateItems::planningThreadTimeTaken, QString::number(state->planning_time_taken));
  SET_ITEM_VALUE(StateTables::modulesState, ModulesStateItems::gbThreadTimeTaken, QString::number(state->gb_time_taken));
  SET_ITEM_VALUE(StateTables::modulesState, ModulesStateItems::visionThreadTimeTaken, QString::number(state->vision_time_taken));
  SET_ITEM_VALUE(StateTables::modulesState, ModulesStateItems::localizerThreadTimeTaken, QString::number(state->localization_time_taken));
  SET_ITEM_VALUE(StateTables::modulesState, ModulesStateItems::userCommThreadTimeTaken, QString::number(state->user_comm_time_taken));
  SET_ITEM_VALUE(StateTables::modulesState, ModulesStateItems::gameCommThreadTimeTaken, QString::number(state->game_comm_time_taken));
}

void RobotState::setRobotState(const team_nust_msgs::TeamNUSTState::ConstPtr& state)
{
    std::string stiffness_state;
  switch (state->stiffness_state) {
    case 0: stiffness_state = "Minimum"; break;
    case 1: stiffness_state = "Maximum"; break;
    case 2: stiffness_state = "Robocup"; break;
    case 3: stiffness_state = "Getup"; break;
    case 4: stiffness_state = "Unknown"; break;
    default: stiffness_state = "Unknown"; break;
  }
  std::string posture_state;
  switch (state->posture_state) {
    case 0: posture_state = "Crouch"; break;
    case 1: posture_state = "Sit"; break;
    case 2: posture_state = "StandZero"; break;
    case 3: posture_state = "Stand"; break;
    case 4: posture_state = "StandHandsBehind"; break;
    case 5: posture_state = "StandWalk"; break;
    case 6: posture_state = "StandKick"; break;
    case 7: posture_state = "GetupReady"; break;
    case 8: posture_state = "FallFront"; break;
    case 9: posture_state = "FallBack"; break;
    case 10: posture_state = "FallingFront"; break;
    case 11: posture_state = "FallingBack"; break;
    case 12: posture_state = "FallSit"; break;
    case 13: posture_state = "DiveInPlace"; break;
    case 14: posture_state = "DiveLeft"; break;
    case 15: posture_state = "DiveRight"; break;
    case 16: posture_state = "DiveSumo"; break;
    case 17: posture_state = "Unknown"; break;
    default: posture_state = "Unknown"; break;
  }
  std::string robot_fallen;
  BOOL_TO_STRING(robot_fallen, state->robot_fallen)

  std::string robot_in_motion;
  BOOL_TO_STRING(robot_in_motion, state->robot_in_motion)

  std::string foot_on_ground;
  switch (state->foot_on_ground) {
    case 0: foot_on_ground = "Left foot"; break;
    case 1: foot_on_ground = "Right foot"; break;
    case 2: foot_on_ground = "Unknown"; break;
    default: foot_on_ground = "Unknown"; break;
  }
  SET_ITEM_VALUE(StateTables::robotState, RobotStateItems::stiffnessState, QString::fromStdString(stiffness_state));
  SET_ITEM_VALUE(StateTables::robotState, RobotStateItems::postureState, QString::fromStdString(posture_state));
  SET_ITEM_VALUE(StateTables::robotState, RobotStateItems::robotFallen, QString::fromStdString(robot_fallen));
  SET_ITEM_VALUE(StateTables::robotState, RobotStateItems::robotInMotion, QString::fromStdString(robot_in_motion));
  SET_ITEM_VALUE(StateTables::robotState, RobotStateItems::footOnGround, QString::fromStdString(foot_on_ground));
}

void RobotState::setGameState(const team_nust_msgs::TeamNUSTState::ConstPtr& state)
{
  SET_ITEM_VALUE(StateTables::gameState, GameStateItems::playerNumber, QString::number(state->player_number));
  SET_ITEM_VALUE(StateTables::gameState, GameStateItems::teamNumber, QString::number(state->team_number));
  SET_ITEM_VALUE(StateTables::gameState, GameStateItems::teamPort, QString::number(state->team_port));

  std::string team_color;
  switch (state->team_color) {
    case 0: team_color = "Blue"; break;
    case 1: team_color = "Red"; break;
    case 2: team_color = "Yellow"; break;
    case 3: team_color = "Black"; break;
    case 4: team_color = "White"; break;
    case 5: team_color = "Green"; break;
    case 6: team_color = "Orange"; break;
    case 7: team_color = "Purple"; break;
    case 8: team_color = "Brown"; break;
    case 9: team_color = "Gray"; break;
    default: team_color = "Unknown"; break;
  }
  SET_ITEM_VALUE(StateTables::gameState, GameStateItems::teamColor, QString::fromStdString(team_color));



  std::string robocup_role;
  switch (state->robocup_role) {
    case 0: robocup_role = "Goal Keeper"; break;
    case 1: robocup_role = "Defender"; break;
    case 2: robocup_role = "Defense Support"; break;
    case 3: robocup_role = "Offense Support"; break;
    case 4: robocup_role = "Attacker"; break;
    case -1: robocup_role = "Unknown"; break;
    default: robocup_role = "Unknown"; break;
  }
  SET_ITEM_VALUE(StateTables::gameState, GameStateItems::robocupRole, QString::fromStdString(robocup_role));

  std::string robot_intention;
  switch (state->robot_intention) {
    case 0: robot_intention = "Nothing in particular"; break;
    case 1: robot_intention = "Wants to be keeper"; break;
    case 2: robot_intention = "Wants to play defense"; break;
    case 3: robot_intention = "Wants to play the ball"; break;
    case 4: robot_intention = "Robot is lost"; break;
    default: robot_intention = "Unknown"; break;
  }
  SET_ITEM_VALUE(StateTables::gameState, GameStateItems::robotIntention, QString::fromStdString(robot_intention));

  std::string whistle_detected;
  BOOL_TO_STRING(whistle_detected, state->whistle_detected)
  SET_ITEM_VALUE(StateTables::gameState, GameStateItems::whistleDetected, QString::fromStdString(whistle_detected));
}

void RobotState::updateLocalizerState(const team_nust_msgs::LocalizationState::ConstPtr& state)
{
  std::string robot_localized, robot_on_side_line, localize_with_last_known, landmarks_found;
  BOOL_TO_STRING(robot_localized, state->robot_localized);
  BOOL_TO_STRING(robot_on_side_line, state->robot_on_side_line);
  BOOL_TO_STRING(localize_with_last_known, state->localize_with_last_known);
  BOOL_TO_STRING(landmarks_found, state->landmarks_found);
  SET_ITEM_VALUE(StateTables::localizerState, LocalizerStateItems::robotLocalized, QString::fromStdString(robot_localized));
  SET_ITEM_VALUE(StateTables::localizerState, LocalizerStateItems::positionConfidence, QString::number(state->position_confidence));
  SET_ITEM_VALUE(StateTables::localizerState, LocalizerStateItems::sideConfidence, QString::number(state->side_confidence));
  SET_ITEM_VALUE(StateTables::localizerState, LocalizerStateItems::robotOnSideLine, QString::fromStdString(robot_on_side_line));
  SET_ITEM_VALUE(StateTables::localizerState, LocalizerStateItems::localizeWithLastKnown, QString::fromStdString(localize_with_last_known));
  SET_ITEM_VALUE(StateTables::localizerState, LocalizerStateItems::landmarksFound, QString::fromStdString(landmarks_found));
}

} // end namespace team_nust_visualizer_plugins

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(team_nust_visualizer_plugins::RobotState, rviz::Panel)

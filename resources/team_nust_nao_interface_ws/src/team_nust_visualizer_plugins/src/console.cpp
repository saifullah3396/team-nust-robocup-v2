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

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QPlainTextEdit>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include <std_msgs/String.h>
#include "console.h"

namespace team_nust_visualizer_plugins
{

Console::Console(QWidget* parent) : rviz::Panel(parent), n_text_lines_(0)
{
  command_publisher_ = nh_.advertise<std_msgs::String>("/team_nust_user_cmds", 1000);
  QHBoxLayout* topic_layout = new QHBoxLayout;
  topic_layout->addWidget(new QLabel("Messages Topic:"));
  input_topic_editor_ = new QLineEdit;
  topic_layout->addWidget(input_topic_editor_);
  text_box_ = new QPlainTextEdit();
  text_box_->setReadOnly(true);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(topic_layout);
  layout->addWidget(text_box_);
  output_cmd_editor_ = new QLineEdit;
  layout->addWidget(output_cmd_editor_);
  setLayout(layout);
  QTimer* display_timer = new QTimer(this);

  // Next we make signal/slot connections.
  connect(input_topic_editor_, SIGNAL(editingFinished()), this, SLOT(updateTopic()));
  connect(output_cmd_editor_, SIGNAL(editingFinished()), this, SLOT(publishCommand()));

  // Start the timer.
  display_timer->start(100);
}

void Console::updateTopic()
{
  setTopic(input_topic_editor_->text());
}

void Console::publishCommand()
{
  output_cmd_editor_->blockSignals(true);
  if (!output_cmd_editor_->text().isEmpty()) {
    std_msgs::String msg;
    msg.data = output_cmd_editor_->text().toStdString();
    command_publisher_.publish(msg);
  }
  output_cmd_editor_->clear();
  output_cmd_editor_->blockSignals(false);
}

// Set the topic name we are publishing to.
void Console::setTopic(const QString& new_topic)
{
  // Only take action if the name has changed.
  if(new_topic != input_topic_)
  {
    input_topic_ = new_topic;
    // If the topic is the empty string, don't publish anything.
    if(input_topic_ != "")
    {
      message_subscriber_ = nh_.subscribe(input_topic_.toStdString(), 10, &Console::handleTopic, this);
    } else {
      message_subscriber_.shutdown();
    }
    Q_EMIT configChanged();
  }
}

void Console::displayMessage()
{
  if (n_text_lines_ >= MAX_TEXT_BUFFER) {
    n_text_lines_ = 0;
    text_box_->clear();
  }
  text_box_->appendPlainText(current_msg_);
  n_text_lines_++;
}

void Console::handleTopic(const std_msgs::String::ConstPtr& msg)
{
  current_msg_ = msg->data.c_str();
  displayMessage();
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void Console::save(rviz::Config config) const
{
  rviz::Panel::save(config);
  config.mapSetValue("Messages Topic", input_topic_);
}

// Load all configuration data for this panel from the given Config object.
void Console::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
  QString topic;
  if(config.mapGetString("Messages Topic", &topic)) {
    input_topic_editor_->setText(topic);
    updateTopic();
  }
}

} // end namespace team_nust_visualizer_plugins

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(team_nust_visualizer_plugins::Console, rviz::Panel)

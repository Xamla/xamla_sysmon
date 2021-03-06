/*
monitor_topic.h

Copyright (c) 2018, Xamla and/or its affiliates.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Xamla and/or its affiliates nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL XAMLA AND/OR ITS AFFILIATES BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef __XamlaSysmonMonitorTopic_H__
#define __XamlaSysmonMonitorTopic_H__

#include <string>
#include <functional>

#include <ros/duration.h>
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>

#include <xamla_sysmon_msgs/HeartBeat.h>

#include "monitor_base.h"

class XamlaSysmonMonitorTopic : public XamlaSysmonMonitorBase
{
public:
  XamlaSysmonMonitorTopic(ros::NodeHandle nh, const std::string& topic_name, const ros::Duration& timeout,
                          std::function<void()> func_callback);
  virtual ~XamlaSysmonMonitorTopic();

  virtual TopicHeartbeatStatus::TopicCode checkTopicStatus();
  virtual std::string generateStatusMessage(TopicHeartbeatStatus::TopicCode topic_status_code);

private:
  void onNewMessage(const topic_tools::ShapeShifter::ConstPtr& msg);

  std::string topic_name;
  ros::Duration timeout;
  ros::Time last_callback;

  TopicHeartbeatStatus::HeartbeatCode curr_heartbeat_status;

  xamla_sysmon_msgs::HeartBeat topic_status_msg;
  bool received_message;

  ros::Subscriber subscriber;
  std::function<void()> new_msg_callback;
};

#endif

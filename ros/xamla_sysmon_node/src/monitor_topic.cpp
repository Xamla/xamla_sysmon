/*
monitor_topic.cpp

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

#include <sstream>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <topic_tools/shape_shifter.h>

#include <xamla_sysmon_msgs/HeartBeat.h>
#include <xamla_sysmon_msgs/statuscodes.h>

#include <xamla_sysmon_node/monitor_topic.h>
#include <xamla_sysmon_node/manager.h>

XamlaSysmonMonitorTopic::XamlaSysmonMonitorTopic(ros::NodeHandle nh, const std::string& monitoring_topic_name,
                                                 const ros::Duration& topic_timeout,
                                                 std::function<void()> new_message_callback)
{
  new_msg_callback = new_message_callback;

  topic_name = monitoring_topic_name;
  timeout = topic_timeout;
  last_callback = ros::Time(0);
  received_message = false;

  curr_heartbeat_status = TopicHeartbeatStatus::HeartbeatCode::TIMEOUT;

  subscriber = nh.subscribe(topic_name, 0, &XamlaSysmonMonitorTopic::onNewMessage, this);
}

XamlaSysmonMonitorTopic::~XamlaSysmonMonitorTopic()
{
  subscriber.shutdown();
}

TopicHeartbeatStatus::TopicCode XamlaSysmonMonitorTopic::checkTopicStatus()
{
  // No new message received since start or reconfiguration
  if (!received_message)
  {
    return TopicHeartbeatStatus::TopicCode::STARTING;
  }

  if (ros::Time::now() - last_callback > timeout)
  {
    if (curr_heartbeat_status != TopicHeartbeatStatus::HeartbeatCode::TIMEOUT)
    {
      curr_heartbeat_status = TopicHeartbeatStatus::HeartbeatCode::TIMEOUT;
      // Running into timeout is a new state, even it was not triggered by an incomming message
      // So we use the new_msg_callback() to inform others about the new state.
      new_msg_callback();
    }
    return TopicHeartbeatStatus::TopicCode::INTERNAL_ERROR;
  }

  // Connection to node exists, so return the status the node reported
  return TopicHeartbeatStatus::TopicCode::GO;
}

std::string XamlaSysmonMonitorTopic::generateStatusMessage(TopicHeartbeatStatus::TopicCode topic_status_code)
{
  std::string msg(TopicHeartbeatStatus::generateMessageText(topic_status_code));
  if (topic_status_code == TopicHeartbeatStatus::TopicCode::INTERNAL_ERROR)
  {
    std::stringstream ss;
    ss << " No update since " << getTimeDiff(ros::Time::now(), last_callback) << " seconds." << std::endl;
    msg += ss.str();
  }

  return msg;
}

// *** Private Functions

void XamlaSysmonMonitorTopic::onNewMessage(const topic_tools::ShapeShifter::ConstPtr& msg)
{
  received_message = true;

  last_callback = ros::Time::now();
}

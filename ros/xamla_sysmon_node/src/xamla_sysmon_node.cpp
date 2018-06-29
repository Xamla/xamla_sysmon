/*
xamla_sysmon_node.cpp

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

#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>
#include <string>
#include <map>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <xamla_sysmon_msgs/SystemStatus.h>
#include <xamla_sysmon_node/monitor_heartbeat.h>
#include <xamla_sysmon_node/manager.h>

int main(int argc, char** argv)
{
  // Maximum valid topic number
  static const int MAX_TOPIC_NUMBER = 100;
  // Dictionary subtree on the parameter server where the list of topics to monitor is stored
  static const std::string DICT_NAME_MONITOR_TOPICS = "monitor_topics";

  ros::init(argc, argv, "xamla_sysmon");

  ros::NodeHandle node_handle("~");

  int number_of_topics = 0;
  if (node_handle.getParam(DICT_NAME_MONITOR_TOPICS + "/number_of_topics", number_of_topics))
  {
    ROS_INFO("Number of topics: %i", number_of_topics);
  }
  else
  {
    ROS_ERROR("Failed to get param 'number_of_topics'");
    ROS_INFO("Number of topics: %i", number_of_topics);
  }

  double publish_frequency = -1;
  if (node_handle.getParam("publish_frequency", publish_frequency))
  {
    ROS_INFO("Publishing with %f Hz.", publish_frequency);
  }
  else
  {
    ROS_ERROR("Failed to get param 'publish_frequency', %f", publish_frequency);
  }

  XamlaSysmonManager manager = XamlaSysmonManager(node_handle, "system_status", publish_frequency);
  ROS_INFO("Start monitor system ...");

  // obtain the configuration for the topics to monitor from parameter server
  for (int i = 0; i < MAX_TOPIC_NUMBER; i++)
  {
    std::stringstream ss;
    ss << DICT_NAME_MONITOR_TOPICS << "/topic" << i;
    std::string topic_id = ss.str();

    std::string topic_name;
    std::string topic_type;
    double topic_timeout;
    if (node_handle.getParam(topic_id + "/name", topic_name) && node_handle.getParam(topic_id + "/type", topic_type) &&
        node_handle.getParam(topic_id + "/timeout", topic_timeout))
    {
      manager.startTopicMonitoring(topic_name, topic_type, topic_timeout);
    }
    else
    {
      // ROS_WARN("Failed to fetch %s", topic_id.c_str());
    }
  }

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    manager.publishSystemStatus();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

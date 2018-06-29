/*
manager.h

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

#ifndef __XamlaSysmonManager_H__
#define __XamlaSysmonManager_H__

#include <string>
#include <memory>

#include <ros/duration.h>
#include <ros/ros.h>

#include <xamla_sysmon_msgs/HeartBeat.h>
#include <xamla_sysmon_msgs/SystemStatus.h>

#include "monitor_base.h"

class XamlaSysmonManager
{
public:
  XamlaSysmonManager(ros::NodeHandle global_node_handle, const std::string& topic_name, double topic_publish_frequency);

  void resetMonitoringList();
  void startTopicMonitoring(const std::string& topic_name, const std::string& topic_type, double timeout);
  void publishSystemStatus();     // obeys rate limit
  void publishSystemStatusNow();  // publish immediately

private:
  std::vector<std::shared_ptr<XamlaSysmonMonitorBase>> monitor_topics;
  xamla_sysmon_msgs::SystemStatus sysstatus_msg;
  ros::NodeHandle node_handle;
  ros::Publisher publisher;

  ros::Time last_status_update;
  double publish_frequency;
};

#endif

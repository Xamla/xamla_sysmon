/*
xamla_sysmon_client.h

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

#ifndef XAMLA_SYSMON_CLIENT_H
#define XAMLA_SYSMON_CLIENT_H

#include "ros/ros.h"
#include "xamla_sysmon_msgs/HeartBeat.h"
#include "string.h"
#include "thread"
#include "string.h"
#include "xamla_sysmon_msgs/statuscodes.h"
#include "mutex"
#include "chrono"

class xamla_sysmon_client
{
public:
  xamla_sysmon_client();
  void start(ros::NodeHandle &node, unsigned int freq, uint64_t timeout);
  void shutdown();
  void updateStatus(TopicHeartbeatStatus::TopicCode new_status, const std::string& details);
private:
  int freq;
  ros::NodeHandle node;
  bool is_start_called;
  bool stop_heartbeat;
  xamla_sysmon_msgs::HeartBeat heartbeat_msg;
  std::thread pub_thread;
  std::chrono::high_resolution_clock::time_point last_update_time;
  std::chrono::milliseconds max_non_update_duration;
  size_t sequence_count;
  void publish_status();
};

#endif // XAMLA_SYSMON_CLIENT_H

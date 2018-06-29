/*
xamla_sysmon_watch.h

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

#ifndef __XamlaSysmonWatch_H__
#define __XamlaSysmonWatch_H__

#include <string>
#include <iostream>
#include <thread>
#include <mutex>

#include <ros/duration.h>
#include <ros/ros.h>

#include <xamla_sysmon_msgs/HeartBeat.h>
#include <xamla_sysmon_msgs/SystemStatus.h>
#include <xamla_sysmon_msgs/statuscodes.h>

#define XAMLA_GLOBAL_STATE_TOPIC "/xamla_sysmon/system_status"

struct XamlaSysmonGlobalState
{
  bool go;
  bool nogo;
  bool only_secondary_error;
  std::string error_message;
  ros::Time time_stamp;
};

class XamlaSysmonWatch
{
public:
  XamlaSysmonWatch(int update_rate_in_hz_ = 10, double time_out_in_s = 0.2);
  ~XamlaSysmonWatch();

  XamlaSysmonGlobalState getGlobalStateSummary();
  xamla_sysmon_msgs::SystemStatus getGlobalState();
  void start();
  void shutdown();

private:
  void fetchGlobalStateThread();
  void handleGlobalStateMessageReceived(const xamla_sysmon_msgs::SystemStatus& message);

  bool received_global_state_;
  bool request_shutdown_;
  bool state_fetcher_running_;
  std::thread state_fetcher_;
  ros::Duration time_out_;
  int update_rate_in_hz_;
  xamla_sysmon_msgs::SystemStatus latest_global_state_;
  std::mutex mutex_;
};

#endif  // __XamlaSysmonWatch_H__

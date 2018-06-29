/*
xamla_sysmon_watch.cpp

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

#include "xamla_sysmon_watch.h"

XamlaSysmonWatch::XamlaSysmonWatch(int update_rate_in_hz, double time_out_in_s)
{
  time_out_ = ros::Duration(time_out_in_s);
  update_rate_in_hz_ = update_rate_in_hz;
}

XamlaSysmonWatch::~XamlaSysmonWatch()
{
  shutdown();
  state_fetcher_.join();
}

xamla_sysmon_msgs::SystemStatus XamlaSysmonWatch::getGlobalState()
{
  return latest_global_state_;
}

XamlaSysmonGlobalState XamlaSysmonWatch::getGlobalStateSummary()
{
  XamlaSysmonGlobalState global_state;
  global_state.go = false;
  global_state.nogo = true;
  global_state.only_secondary_error = false;
  global_state.error_message = "Invalid State";
  global_state.time_stamp = ros::Time(0);

  mutex_.lock();
  if (!state_fetcher_running_) {
    global_state.nogo = true;
    global_state.error_message = "The state fetcher thread is not running.";
  }
  else if (!received_global_state_)
  {
    global_state.nogo = true;
    global_state.error_message = "XamlaSysmonWatch has not received any global state updates yet";
  }
  else if (ros::Time::now() - latest_global_state_.header.stamp >= time_out_)
  {
    global_state.nogo = true;
    global_state.error_message = "The state of XamlaSysmonWatch is outdated.";
    global_state.time_stamp = ros::Time::now();
  }
  else
  {
    switch (latest_global_state_.system_status)
    {
      case 0:
      {
        global_state.go = true;
        global_state.nogo = false;
        global_state.error_message = "";
        global_state.time_stamp = latest_global_state_.header.stamp;
        break;
      }
      case 1:
      {
        global_state.nogo = true;
        global_state.only_secondary_error = true;
        global_state.error_message = "There are only secondary erros.";
        global_state.time_stamp = latest_global_state_.header.stamp;
        break;
      }
      default:
      {
        global_state.nogo = true;
        global_state.time_stamp = latest_global_state_.header.stamp;
        std::stringstream ss;
        if (latest_global_state_.system_status & (1 << 3))
        {
          ss << "At least one node is in INVALID state. ";
        }
        if (latest_global_state_.system_status & (1 << 2))
        {
          ss << "At least one node is in EMERGENCY_STOP state. ";
        }
        if (latest_global_state_.system_status & (1 << 1))
        {
          ss << "At least one node is in INTERNAL_ERROR state. ";
        }
        if (latest_global_state_.system_status & (1 << 0))
        {
          ss << "At least one node is in SECONDARY_ERROR state. ";
        }
        ss << std::endl;
        global_state.error_message = ss.str();
      }
    }
  }
  mutex_.unlock();

  return global_state;
}

void XamlaSysmonWatch::shutdown()
{
  mutex_.lock();
  request_shutdown_ = true;
  mutex_.unlock();
}

void XamlaSysmonWatch::start()
{
  if (!state_fetcher_running_) {
    state_fetcher_ = std::thread(&XamlaSysmonWatch::fetchGlobalStateThread, this);
  }
}

void XamlaSysmonWatch::fetchGlobalStateThread()
{
  state_fetcher_running_ = true;

  int argc = 0;
  ros::init(argc, nullptr, "xamla_sysmon_watch");
  ros::NodeHandle node_handle("~");

  ros::Subscriber subscriber =
      node_handle.subscribe(XAMLA_GLOBAL_STATE_TOPIC, 0, &XamlaSysmonWatch::handleGlobalStateMessageReceived, this);
  ROS_INFO("Subscribed to %s", XAMLA_GLOBAL_STATE_TOPIC);

  ros::Rate loop_rate(update_rate_in_hz_);
  while (ros::ok() && !request_shutdown_)
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  if (!request_shutdown_) {
    ROS_FATAL("Fetch thread has exited ungracefully.");
  }

  node_handle.shutdown();
  ros::shutdown();

  state_fetcher_running_ = false;
}

void XamlaSysmonWatch::handleGlobalStateMessageReceived(const xamla_sysmon_msgs::SystemStatus& message)
{
  mutex_.lock();
  received_global_state_ = true;
  latest_global_state_ = message;
  mutex_.unlock();
}

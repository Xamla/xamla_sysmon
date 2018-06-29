/*
testnode.cpp

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

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <xamla_sysmon_msgs/HeartBeat.h>
#include <xamla_sysmon_msgs/statuscodes.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "testnode");

  ros::NodeHandle node_handle("~");

  ros::Publisher heartbeat = node_handle.advertise<xamla_sysmon_msgs::HeartBeat>("heartbeat", 1);
  ros::Rate loop_rate(1);

  int count = 0;
  while (ros::ok())
  {
    xamla_sysmon_msgs::HeartBeat msg;

    msg.header.stamp = ros::Time::now();

    if (count < 8)
    {
      msg.status = static_cast<int>(TopicHeartbeatStatus::TopicCode::STARTING);
      msg.details = TopicHeartbeatStatus::generateMessageText(TopicHeartbeatStatus::intToStatusCode(msg.status));
    }
    else
    {
      msg.status = static_cast<int>(TopicHeartbeatStatus::TopicCode::GO);
      msg.details = "";
    }

    // std::stringstream ss;
    // ss << "hello world " << count;
    // msg.data = ss.str();

    // ROS_INFO("%s", msg.data.c_str());

    heartbeat.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
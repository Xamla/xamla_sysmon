/*
xamla_sysmon_client.cpp

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

#include "xamla_sysmon_client/xamla_sysmon_client.h"

xamla_sysmon_client::xamla_sysmon_client()
{
  this->is_start_called = false;
  this->stop_heartbeat = false;
  last_update_time = std::chrono::high_resolution_clock::now();
}

void xamla_sysmon_client::start(ros::NodeHandle &node, unsigned int freq, uint64_t timeout) {
  if (this->is_start_called == false)
    this->is_start_called = true;
  else //if (this->is_start_called == true)
  {
    ROS_WARN("Heartbeat: start already called once, nothing to do");
    return;
  }

  this->node= node;

  //checking for +ve frequency
  if(freq != 0)
    this->freq = freq;
  else
  {
    this->freq = 10;
    ROS_WARN("heatbeat freq cannot be 0");
  }
  ROS_INFO("[Heartbeat] publishing frequency: %d", this->freq);

  this->sequence_count = 0;
  this->max_non_update_duration = std::chrono::milliseconds(timeout);

  this->heartbeat_msg.header.seq = sequence_count++;
  this->heartbeat_msg.header.stamp = ros::Time::now();
  this->heartbeat_msg.status = (int)TopicHeartbeatStatus::TopicCode::STARTING;
  this->heartbeat_msg.details = "Node starting";

  //openning new thread to publish state
  this->pub_thread = std::thread(&xamla_sysmon_client::publish_status, this);
}

void xamla_sysmon_client::publish_status()
{
  //ros publisher
  ROS_INFO("starting heartbeat publishing loop");
  ros::Publisher status_pub = this->node.advertise<xamla_sysmon_msgs::HeartBeat>(ros::this_node::getName()+"/heartbeat", 100);
  ros::Rate loop_rate(this->freq);

  while(ros::ok())
  {
    //if shutdown was called
    if (this->stop_heartbeat == true)
    {
      break;
    }

    this->heartbeat_msg.header.seq = sequence_count++;
    this->heartbeat_msg.header.stamp = ros::Time::now();

    std::mutex my_mutex;
    auto current_time = std::chrono::high_resolution_clock::now();
    std::cout<<std::chrono::duration_cast<std::chrono::milliseconds>(current_time-last_update_time).count()<<std::endl;
    if(std::chrono::duration_cast<std::chrono::milliseconds>(current_time-last_update_time).count() > max_non_update_duration.count() &&
       this->heartbeat_msg.status != int(TopicHeartbeatStatus::TopicCode::STARTING))
    {
      std::lock_guard<std::mutex> lock(my_mutex);
      this->heartbeat_msg.status = int(TopicHeartbeatStatus::TopicCode::INTERNAL_ERROR);
      this->heartbeat_msg.details = "Watchdog: the duration between two updates was to long, error";
      status_pub.publish(this->heartbeat_msg);
    }
    else
    {
      //thread lock guard
      std::lock_guard<std::mutex> lock(my_mutex);
      status_pub.publish(this->heartbeat_msg);
    //ROS_INFO("published a heartbeat");
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  //shutting down
  //publisher gets out of scope and terminate no need to shut it down
  ROS_INFO("Heartbeat: stopping heartbeat");
}

void xamla_sysmon_client::shutdown()
{
  if (this->is_start_called == true)
    this->stop_heartbeat = true;
  else //(this->is_start_called == false)
    ROS_WARN("Heartbeat wasn't running");
}

void xamla_sysmon_client::updateStatus(TopicHeartbeatStatus::TopicCode new_status, const std::string &details)
{
  //lock guard
  std::mutex my_mutex;
  std::lock_guard<std::mutex> lock(my_mutex);
  //setting heartbeat msg fields
  this->heartbeat_msg.status = (int)new_status;
  this->heartbeat_msg.details = details;
  last_update_time = std::chrono::high_resolution_clock::now();
}

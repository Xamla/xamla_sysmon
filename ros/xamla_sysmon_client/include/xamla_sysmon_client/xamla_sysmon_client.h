#ifndef XAMLA_SYSMON_CLIENT_H
#define XAMLA_SYSMON_CLIENT_H

#include "ros/ros.h"
#include "xamla_sysmon_msgs/HeartBeat.h"
#include "string.h"
#include "thread"
#include "string.h"

//TODO replace enum from the TopicCode class enum from statuscode.h
enum heartbeat_status {GO = 0, SECONDARY_ERROR = 1, ERROR = 2, EMERGENCY_STOP = 4, UNKOWN_ERROR = 8};

class xamla_sysmon_client
{
public:
  ros::NodeHandle node;
  int freq;
  xamla_sysmon_msgs::HeartBeat heartbeat_msg;
  xamla_sysmon_client();
  void start(ros::NodeHandle &node, int freq);
  void shutdown();
  int32_t updateStatus(heartbeat_status new_status, std::string details); //should it return heartbeat status object instead
private:
  void publish_status();
  std::thread pub_thread;
};

#endif // XAMLA_SYSMON_CLIENT_H

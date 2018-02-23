#ifndef XAMLA_SYSMON_CLIENT_H
#define XAMLA_SYSMON_CLIENT_H

#include "ros/ros.h"
#include "xamla_sysmon_msgs/HeartBeat.h"
#include "string.h"
#include "thread"
#include "string.h"

//TODO which states are right
//TODO maybe make it class to be type safe and then cast it to int for the message
enum heartbeat_status {GO = 0, SECONDARY_ERROR = 1, ERROR = 2, EMERGENCY_STOP = 4, UNKOWN_ERROR = 8};
//enum heartbeat_status {STARTING, GO, INTERNAL_ERROR, EMERGENCY_STOP, SECONDARY_ERROR};

class xamla_sysmon_client
{
public:
  ros::NodeHandle node;
  int freq;
  ros::Time last_publish_time; //TODO probably not needed - delete it
  heartbeat_status last_publish_status; //TODO probably not needed - delete it
  xamla_sysmon_msgs::HeartBeat heartbeat_msg;
 //TODO erase status and details and change them directly in the msg
  heartbeat_status status;
  std::string details;

  xamla_sysmon_client();
  void start(ros::NodeHandle &node, int freq);
  void shutdown();
  int32_t updateStatus(heartbeat_status new_status, std::string details); //should it return heartbeat status oject instead
private:
  void publish_status();
  std::thread pub_thread;
};

#endif // XAMLA_SYSMON_CLIENT_H

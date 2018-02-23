#ifndef XAMLA_SYSMON_CLIENT_H
#define XAMLA_SYSMON_CLIENT_H

#include "ros/ros.h"
#include "xamla_sysmon_msgs/HeartBeat.h"
#include "string.h"
#include "thread"
#include "string.h"
#include "xamla_sysmon_msgs/statuscodes.h"
#include "mutex"

class xamla_sysmon_client
{
public:
  xamla_sysmon_client();
  void start(ros::NodeHandle &node, unsigned int freq);
  void shutdown();
  void updateStatus(TopicHeartbeatStatus::TopicCode new_status, std::string details);
private:
  int freq;
  ros::NodeHandle node;
  bool is_start_called;
  bool stop_heartbeat;
  xamla_sysmon_msgs::HeartBeat heartbeat_msg;
  std::thread pub_thread;
  void publish_status();
};

#endif // XAMLA_SYSMON_CLIENT_H

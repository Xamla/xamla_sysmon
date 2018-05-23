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
  void updateStatus(TopicHeartbeatStatus::TopicCode new_status, const std::string details);
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

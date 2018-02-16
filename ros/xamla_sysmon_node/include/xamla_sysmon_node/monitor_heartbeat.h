#ifndef __XamlaSysmonMonitorHeartbeat_H__
#define __XamlaSysmonMonitorHeartbeat_H__

#include <string>
#include <functional>

#include <ros/duration.h>
#include <ros/ros.h>

#include <xamla_sysmon_msgs/HeartBeat.h>
#include <xamla_sysmon_msgs/statuscodes.h>

#include "monitor_base.h"

class XamlaSysmonMonitorHeartbeat : public XamlaSysmonMonitorBase
{
public:
  XamlaSysmonMonitorHeartbeat(ros::NodeHandle nh, const std::string& monitoring_topic_name,
                              const ros::Duration& topic_timeout, std::function<void()> func_callback);
  virtual ~XamlaSysmonMonitorHeartbeat();

  TopicHeartbeatStatus::TopicCode checkTopicStatus();
  std::string generateStatusMessage(TopicHeartbeatStatus::TopicCode topic_status_code);

private:
  void onNewMessage(const xamla_sysmon_msgs::HeartBeat& msg);

  std::string topic_name;
  ros::Duration timeout;
  ros::Time last_callback;

  TopicHeartbeatStatus::HeartbeatCode curr_heartbeat_status;

  xamla_sysmon_msgs::HeartBeat topic_status_msg;

  ros::Subscriber subscriber;
  std::function<void()> new_msg_callback;
};

#endif

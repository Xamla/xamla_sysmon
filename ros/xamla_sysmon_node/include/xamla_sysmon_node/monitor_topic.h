#ifndef __XamlaSysmonMonitorTopic_H__
#define __XamlaSysmonMonitorTopic_H__

#include <string>
#include <functional>

#include <ros/duration.h>
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>

#include <xamla_sysmon_msgs/HeartBeat.h>

#include "monitor_base.h"

class XamlaSysmonMonitorTopic : public XamlaSysmonMonitorBase
{
public:
  XamlaSysmonMonitorTopic(ros::NodeHandle nh, const std::string& topic_name, const ros::Duration& timeout,
                          std::function<void()> func_callback);
  virtual ~XamlaSysmonMonitorTopic();

  virtual TopicHeartbeatStatus::TopicCode checkTopicStatus();
  virtual std::string generateStatusMessage(TopicHeartbeatStatus::TopicCode topic_status_code);

private:
  void onNewMessage(const topic_tools::ShapeShifter::ConstPtr& msg);

  std::string topic_name;
  ros::Duration timeout;
  ros::Time last_callback;

  TopicHeartbeatStatus::HeartbeatCode curr_heartbeat_status;

  xamla_sysmon_msgs::HeartBeat topic_status_msg;
  bool received_message;

  ros::Subscriber subscriber;
  std::function<void()> new_msg_callback;
};

#endif

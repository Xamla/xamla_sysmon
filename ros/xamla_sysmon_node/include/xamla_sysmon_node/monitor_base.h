#ifndef __XamlaSysmonMonitorBase_H__
#define __XamlaSysmonMonitorBase_H__

#include <string>
#include <functional>

#include <ros/duration.h>
#include <ros/ros.h>

#include <xamla_sysmon_msgs/statuscodes.h>

class XamlaSysmonMonitorBase
{
public:
  virtual TopicHeartbeatStatus::TopicCode checkTopicStatus() = 0;
  virtual std::string generateStatusMessage(TopicHeartbeatStatus::TopicCode topic_status_code) = 0;

protected:
  virtual double getTimeDiff(const ros::Time& last, const ros::Time& now, int precision = 1);
};

#endif

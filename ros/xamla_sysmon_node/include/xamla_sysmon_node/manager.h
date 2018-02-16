#ifndef __XamlaSysmonManager_H__
#define __XamlaSysmonManager_H__

#include <string>
#include <memory>

#include <ros/duration.h>
#include <ros/ros.h>

#include <xamla_sysmon_msgs/HeartBeat.h>
#include <xamla_sysmon_msgs/SystemStatus.h>

#include "monitor_base.h"

class XamlaSysmonManager
{
public:
  XamlaSysmonManager(ros::NodeHandle global_node_handle, const std::string& topic_name, double topic_publish_frequency);

  void resetMonitoringList();
  void startTopicMonitoring(const std::string& topic_name, const std::string& topic_type, double timeout);
  void publishSystemStatus();     // obeys rate limit
  void publishSystemStatusNow();  // publish immediately

private:
  std::vector<std::shared_ptr<XamlaSysmonMonitorBase>> monitor_topics;
  xamla_sysmon_msgs::SystemStatus sysstatus_msg;
  ros::NodeHandle node_handle;
  ros::Publisher publisher;

  ros::Time last_status_update;
  double publish_frequency;
};

#endif

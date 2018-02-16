#include <sstream>
#include <vector>
#include <string>
#include <memory>
#include <functional>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>

#include <xamla_sysmon_msgs/HeartBeat.h>
#include <xamla_sysmon_msgs/statuscodes.h>

#include <xamla_sysmon_node/monitor_base.h>
#include <xamla_sysmon_node/monitor_topic.h>
#include <xamla_sysmon_node/monitor_heartbeat.h>
#include <xamla_sysmon_node/manager.h>

XamlaSysmonManager::XamlaSysmonManager(ros::NodeHandle global_node_handle, const std::string& topic_name,
                                       double topic_publish_frequency)
{
  node_handle = global_node_handle;
  publisher = node_handle.advertise<xamla_sysmon_msgs::SystemStatus>(topic_name, 1);
  publish_frequency = topic_publish_frequency;
  last_status_update = ros::Time(0);
}

void XamlaSysmonManager::resetMonitoringList()
{
  monitor_topics.clear();
  last_status_update = ros::Time(0);
}

void XamlaSysmonManager::startTopicMonitoring(const std::string& topic_name, const std::string& topic_type,
                                              double timeout)
{
  if (topic_type == "heartbeat")
  {
    monitor_topics.push_back(std::shared_ptr<XamlaSysmonMonitorBase>(
        new XamlaSysmonMonitorHeartbeat(node_handle, topic_name, ros::Duration(timeout),
                                        std::bind(&XamlaSysmonManager::publishSystemStatusNow, this))));
    ROS_INFO("Start monitoring heartbeat topic %s, timeout: %f", topic_name.c_str(), timeout);
    sysstatus_msg.topics.push_back(topic_name);
    sysstatus_msg.err_code.push_back(1);  // TODO
    sysstatus_msg.topic_msg.push_back("");
  }
  else if (topic_type == "default")
  {
    monitor_topics.push_back(std::shared_ptr<XamlaSysmonMonitorBase>(
        new XamlaSysmonMonitorTopic(node_handle, topic_name, ros::Duration(timeout),
                                    std::bind(&XamlaSysmonManager::publishSystemStatusNow, this))));
    ROS_INFO("Start monitoring heartbeat topic %s, timeout: %f", topic_name.c_str(), timeout);
    sysstatus_msg.topics.push_back(topic_name);
    sysstatus_msg.err_code.push_back(1);  // TODO
    sysstatus_msg.topic_msg.push_back("");
  }
  else
  {
    ROS_ERROR("Unknown monitoring type %s for topic %s, skipping entry\n", topic_name.c_str(), topic_type.c_str());
  }
}

void XamlaSysmonManager::publishSystemStatus()
{
  // Check each topic state. Without this check timeouts get detected only when
  // the next normal global status update is performed
  // When a timeout is detected checkTopicStatus() triggers PublishSystemStatusNow()
  // on its own
  for (int i = 0; i < monitor_topics.size(); i++)
  {
    monitor_topics[i]->checkTopicStatus();
  }

  if ((ros::Time::now() - last_status_update).toSec() >= 1.0 / publish_frequency)
  {
    publishSystemStatusNow();
    last_status_update = ros::Time::now();
  }
}

void XamlaSysmonManager::publishSystemStatusNow()
{
  uint32_t system_status = 0;
  for (int i = 0; i < monitor_topics.size(); i++)
  {
    TopicHeartbeatStatus::TopicCode topic_status = monitor_topics[i]->checkTopicStatus();
    sysstatus_msg.err_code[i] = static_cast<int>(topic_status);
    if (!TopicHeartbeatStatus::checkStatusOK(topic_status))
    {
      sysstatus_msg.topic_msg[i] = monitor_topics[i]->generateStatusMessage(topic_status);
      // TODO: Remove hardcoded bit positions and review meaning of the different bits
      if (topic_status == TopicHeartbeatStatus::TopicCode::SECONDARY_ERROR)
      {
        system_status |= 1 << 0;
      }
      else if (topic_status == TopicHeartbeatStatus::TopicCode::STARTING ||
               topic_status == TopicHeartbeatStatus::TopicCode::INTERNAL_ERROR)
      {
        system_status |= 1 << 1;
      }
      else if (topic_status == TopicHeartbeatStatus::TopicCode::EMERGENCY_STOP)
      {
        system_status |= 1 << 2;
      }
      else
      {
        // other error in topic
        system_status |= 1 << 3;
      }
    }
    else
    {
      sysstatus_msg.topic_msg[i] = "";
    }
  }

  sysstatus_msg.header.stamp = ros::Time::now();
  sysstatus_msg.system_status = system_status;

  publisher.publish(sysstatus_msg);
}

#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>
#include <string>
#include <map>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <xamla_sysmon_msgs/SystemStatus.h>
#include <xamla_sysmon_node/monitor_heartbeat.h>
#include <xamla_sysmon_node/manager.h>

int main(int argc, char** argv)
{
  // Maximum valid topic number
  static const int MAX_TOPIC_NUMBER = 100;
  // Dictionary subtree on the parameter server where the list of topics to monitor is stored
  static const std::string DICT_NAME_MONITOR_TOPICS = "monitor_topics";

  ros::init(argc, argv, "xamla_sysmon");

  ros::NodeHandle node_handle("~");

  int number_of_topics = 0;
  if (node_handle.getParam(DICT_NAME_MONITOR_TOPICS + "/number_of_topics", number_of_topics))
  {
    ROS_INFO("Number of topics: %i", number_of_topics);
  }
  else
  {
    ROS_ERROR("Failed to get param 'number_of_topics'");
    ROS_INFO("Number of topics: %i", number_of_topics);
  }

  double publish_frequency = -1;
  if (node_handle.getParam("publish_frequency", publish_frequency))
  {
    ROS_INFO("Publishing with %f Hz.", publish_frequency);
  }
  else
  {
    ROS_ERROR("Failed to get param 'publish_frequency', %f", publish_frequency);
  }

  XamlaSysmonManager manager = XamlaSysmonManager(node_handle, "system_status", publish_frequency);
  ROS_INFO("Start monitor system ...");

  // obtain the configuration for the topics to monitor from parameter server
  for (int i = 0; i < MAX_TOPIC_NUMBER; i++)
  {
    std::stringstream ss;
    ss << DICT_NAME_MONITOR_TOPICS << "/topic" << i;
    std::string topic_id = ss.str();

    std::string topic_name;
    std::string topic_type;
    double topic_timeout;
    if (node_handle.getParam(topic_id + "/name", topic_name) && node_handle.getParam(topic_id + "/type", topic_type) &&
        node_handle.getParam(topic_id + "/timeout", topic_timeout))
    {
      manager.startTopicMonitoring(topic_name, topic_type, topic_timeout);
    }
    else
    {
      // ROS_WARN("Failed to fetch %s", topic_id.c_str());
    }
  }

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    manager.publishSystemStatus();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

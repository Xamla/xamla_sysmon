#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <functional>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <xamla_sysmon_msgs/HeartBeat.h>
#include <xamla_sysmon_msgs/statuscodes.h>

#include <xamla_sysmon_node/monitor_heartbeat.h>
#include <xamla_sysmon_node/manager.h>

XamlaSysmonMonitorHeartbeat::XamlaSysmonMonitorHeartbeat(ros::NodeHandle nh, const std::string& monitoring_topic_name,
                                                         const ros::Duration& topic_timeout,
                                                         std::function<void()> new_message_callback)
{
  new_msg_callback = new_message_callback;

  topic_name = monitoring_topic_name;
  timeout = topic_timeout;
  last_callback = ros::Time::now();

  curr_heartbeat_status = TopicHeartbeatStatus::HeartbeatCode::TIMEOUT;
  topic_status_msg.status = static_cast<int>(TopicHeartbeatStatus::TopicCode::STARTING);

  subscriber = nh.subscribe(topic_name, 0, &XamlaSysmonMonitorHeartbeat::onNewMessage, this);
}

XamlaSysmonMonitorHeartbeat::~XamlaSysmonMonitorHeartbeat()
{
  subscriber.shutdown();
}

TopicHeartbeatStatus::TopicCode XamlaSysmonMonitorHeartbeat::checkTopicStatus()
{
  if (ros::Time::now() - last_callback > timeout)
  {
    if (curr_heartbeat_status != TopicHeartbeatStatus::HeartbeatCode::TIMEOUT)
    {
      curr_heartbeat_status = TopicHeartbeatStatus::HeartbeatCode::TIMEOUT;
      // Running into timeout is a new state, even it was not triggered by an incomming message
      // So we use the new_msg_callback() to inform others about the new state.
      new_msg_callback();
    }
    return TopicHeartbeatStatus::TopicCode::INTERNAL_ERROR;
  }
  else
  {
    if (curr_heartbeat_status != TopicHeartbeatStatus::HeartbeatCode::ESTABLISHED)
    {
      curr_heartbeat_status = TopicHeartbeatStatus::HeartbeatCode::ESTABLISHED;
      // Running into timeout is a new state, even it was not triggered by an incomming message
      // So we use the new_msg_callback() to inform others about the new state.
      new_msg_callback();
    }
  }

  // Connection to node exists, so return the status the node reported
  if (curr_heartbeat_status == TopicHeartbeatStatus::HeartbeatCode::ESTABLISHED)
  {
    return TopicHeartbeatStatus::intToStatusCode(topic_status_msg.status);
  }

  ROS_WARN("Topic %s is in invalid state.", topic_name.c_str());
  return TopicHeartbeatStatus::TopicCode::INVALID;
}

std::string XamlaSysmonMonitorHeartbeat::generateStatusMessage(TopicHeartbeatStatus::TopicCode topic_status_code)
{
  std::string msg = TopicHeartbeatStatus::generateMessageText(topic_status_code);
  // Some communication error with the node
  // So we have to generate the error message here and can't just use the message as reported by the node
  if (curr_heartbeat_status != TopicHeartbeatStatus::HeartbeatCode::ESTABLISHED)
  {
    if (curr_heartbeat_status == TopicHeartbeatStatus::HeartbeatCode::TIMEOUT)
    {
      std::stringstream ss;
      ss << " No update since " << getTimeDiff(ros::Time::now(), last_callback) << " seconds." << std::endl;
      msg += ss.str();
    }
  }
  // communication with node is working, just use the message reported by node
  else
  {
    std::stringstream ss;
    ss << " (" << topic_status_msg.details << ")";
    msg += ss.str();
  }

  return msg;
}

// *** Private Functions

void XamlaSysmonMonitorHeartbeat::onNewMessage(const xamla_sysmon_msgs::HeartBeat& msg)
{
  last_callback = ros::Time::now();

  if (topic_status_msg.status != msg.status || topic_status_msg.details != msg.details)
  {
    topic_status_msg = msg;
    std::cout << "Heartbeat current topic status: " << static_cast<int>(curr_heartbeat_status) << std::endl;
    new_msg_callback();
  }
}

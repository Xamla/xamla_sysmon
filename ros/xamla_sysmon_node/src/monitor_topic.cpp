#include <sstream>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <topic_tools/shape_shifter.h>

#include <xamla_sysmon_msgs/HeartBeat.h>
#include <xamla_sysmon_msgs/statuscodes.h>

#include <xamla_sysmon_node/monitor_topic.h>
#include <xamla_sysmon_node/manager.h>

XamlaSysmonMonitorTopic::XamlaSysmonMonitorTopic(ros::NodeHandle nh, const std::string& monitoring_topic_name,
                                                 const ros::Duration& topic_timeout,
                                                 std::function<void()> new_message_callback)
{
  new_msg_callback = new_message_callback;

  topic_name = monitoring_topic_name;
  timeout = topic_timeout;
  last_callback = ros::Time(0);
  received_message = false;

  curr_heartbeat_status = TopicHeartbeatStatus::HeartbeatCode::TIMEOUT;

  subscriber = nh.subscribe(topic_name, 0, &XamlaSysmonMonitorTopic::onNewMessage, this);
}

XamlaSysmonMonitorTopic::~XamlaSysmonMonitorTopic()
{
  subscriber.shutdown();
}

TopicHeartbeatStatus::TopicCode XamlaSysmonMonitorTopic::checkTopicStatus()
{
  // No new message received since start or reconfiguration
  if (!received_message)
  {
    return TopicHeartbeatStatus::TopicCode::STARTING;
  }

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

  // Connection to node exists, so return the status the node reported
  return TopicHeartbeatStatus::TopicCode::GO;
}

std::string XamlaSysmonMonitorTopic::generateStatusMessage(TopicHeartbeatStatus::TopicCode topic_status_code)
{
  std::string msg(TopicHeartbeatStatus::generateMessageText(topic_status_code));
  if (topic_status_code == TopicHeartbeatStatus::TopicCode::INTERNAL_ERROR)
  {
    std::stringstream ss;
    ss << " No update since " << getTimeDiff(ros::Time::now(), last_callback) << " seconds." << std::endl;
    msg += ss.str();
  }

  return msg;
}

// *** Private Functions

void XamlaSysmonMonitorTopic::onNewMessage(const topic_tools::ShapeShifter::ConstPtr& msg)
{
  received_message = true;

  last_callback = ros::Time::now();
}

#include <sstream>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <xamla_sysmon_msgs/HeartBeat.h>
#include <xamla_sysmon_msgs/statuscodes.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "testnode");

  ros::NodeHandle node_handle("~");

  ros::Publisher heartbeat = node_handle.advertise<xamla_sysmon_msgs::HeartBeat>("heartbeat", 1);
  ros::Rate loop_rate(1);

  int count = 0;
  while (ros::ok())
  {
    xamla_sysmon_msgs::HeartBeat msg;

    msg.header.stamp = ros::Time::now();

    if (count < 8)
    {
      msg.status = static_cast<int>(TopicHeartbeatStatus::TopicCode::STARTING);
      msg.details = TopicHeartbeatStatus::generateMessageText(TopicHeartbeatStatus::intToStatusCode(msg.status));
    }
    else
    {
      msg.status = static_cast<int>(TopicHeartbeatStatus::TopicCode::GO);
      msg.details = "";
    }

    // std::stringstream ss;
    // ss << "hello world " << count;
    // msg.data = ss.str();

    // ROS_INFO("%s", msg.data.c_str());

    heartbeat.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
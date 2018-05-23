#include "ros/ros.h"
#include "std_msgs/String.h"
#include "xamla_sysmon_client/xamla_sysmon_client.h"
#include "string"

#include <sstream>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Rate loop_rate(10);
  int count = 0;

  //declaring heartbeat -----------------------------------------
  TopicHeartbeatStatus::TopicCode my_status;
  std::string my_details;
  xamla_sysmon_client heartbeat_client;
  heartbeat_client.start(n, 12, 900);  //passing node handle and specifying freq and max duration between status updates in milliseconds
  // ------------------------------------------------------------

  while (ros::ok())
  {
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world thread " << count;
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());
    chatter_pub.publish(msg);

    //testing all states -------------------------------------------------------------------------------------------
    //changing state every 10 messages - then closing down heartbeat and publishing extra 30 messages
    //states GO = 0, STARTING = 100, INTERNAL_ERROR = 200, EMERGENCY_STOP = 300, SECONDARY_ERROR = 400, INVALID = -1
    if      (count == 0)  { my_status = TopicHeartbeatStatus::TopicCode::GO;              my_details = "GO";
                            ROS_INFO("changed state to GO");                              heartbeat_client.updateStatus(my_status, my_details); }
    else if (count == 10) { my_status = TopicHeartbeatStatus::TopicCode::STARTING;        my_details = "STARTING";
                            ROS_INFO("changed state to STARTING");                        heartbeat_client.updateStatus(my_status, my_details); }
    else if (count == 20) { my_status = TopicHeartbeatStatus::TopicCode::INTERNAL_ERROR;  my_details = "INTERNAL_ERROR";
                            ROS_INFO("changed state to INTERNAL_ERROR");                  heartbeat_client.updateStatus(my_status, my_details); }
    else if (count == 30) { my_status = TopicHeartbeatStatus::TopicCode::EMERGENCY_STOP;  my_details = "EMERGENCY_STOP";
                            ROS_INFO("changed state to EMERGENCY_STOP");                  heartbeat_client.updateStatus(my_status, my_details); }
    else if (count == 40) { my_status = TopicHeartbeatStatus::TopicCode::SECONDARY_ERROR; my_details = "SECONDARY_ERROR";
                            ROS_INFO("changed state to SECONDARY_ERROR");                 heartbeat_client.updateStatus(my_status, my_details); }
    else if (count == 50) { my_status = TopicHeartbeatStatus::TopicCode::INVALID;         my_details = "INVALID";
                            ROS_INFO("changed state to INVALID");                         heartbeat_client.updateStatus(my_status, my_details); }
    else if (count == 60) { heartbeat_client.shutdown(); } //heartbeat stop but topic still running
    else if (count == 100) { break; }
    //--------------------------------------------------------------------------------------------------------------

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }

  ROS_INFO("finished test - shutting down");

  return 0;
}

#include "xamla_sysmon_client/xamla_sysmon_client.h"

xamla_sysmon_client::xamla_sysmon_client()
{
}

void xamla_sysmon_client::start(ros::NodeHandle &node, int freq) {
  //TODO check if already called
  this->node= node;

  if(freq > 0)
    this->freq = freq;
  else
  {
    this->freq = 10;
    ROS_WARN("heatbeat freq cannot be 0 or less; default to 10");
  }
  ROS_INFO("[Heartbeat] publishing frequency: %d", this->freq);

  this->heartbeat_msg.header.stamp = ros::Time::now();
  this->heartbeat_msg.status = GO;
  this->heartbeat_msg.details = "Node starting";

  //openning new thread to publish state
  this->pub_thread = std::thread(&xamla_sysmon_client::publish_status, this);
}

void xamla_sysmon_client::publish_status()
{
  //TODO add mutex to guard thread
  //ros publisher
  ROS_INFO("starting heartbeat publishing loop");
  ros::Publisher status_pub = this->node.advertise<xamla_sysmon_msgs::HeartBeat>(ros::this_node::getName()+"/heartbeat", 100);
  ros::Rate loop_rate(this->freq);

  while(ros::ok())
  {
    status_pub.publish(this->heartbeat_msg);
    //ROS_INFO("published a heartbeat");
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void xamla_sysmon_client::shutdown()
{
  //TODO shutdown the thread here
}

int32_t xamla_sysmon_client::updateStatus(heartbeat_status new_status, std::string details)
{
  //TODO check if status is null or change it to type safe
  this->heartbeat_msg.status = new_status;
  this->heartbeat_msg.details = details;
}

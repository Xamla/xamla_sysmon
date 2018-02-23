#include "xamla_sysmon_client/xamla_sysmon_client.h"

xamla_sysmon_client::xamla_sysmon_client()
{
}

void xamla_sysmon_client::start(ros::NodeHandle &node, int freq) {
  this->node= node;

  if(freq > 0)
    this->freq = freq;
  else
    this->freq = freq; //TODO print warn

  this->last_publish_time = ros::Time::now();
  this->status = GO;
  ROS_INFO("[Heartbeat] publishing frequency: %d", freq);
  //first message
  this->heartbeat_msg.header.stamp = this->last_publish_time;
  //this->heartbeat_msg.header.seq = 1;
  this->heartbeat_msg.status = this->status;
  this->details = "First heartbeart";
  this->heartbeat_msg.details = this->details;

  ROS_INFO("before thread");

  //openning new thread to publish state
  this->pub_thread = std::thread(&xamla_sysmon_client::publish_status, this);

  ROS_INFO("after thread");

  //while(ros::ok()); //TODO remove this and keep thread alive without it
}

void xamla_sysmon_client::publish_status()
{
  //ros publisher
  ROS_INFO("starting heartbeat publishing loop");
  //TODO add namespace or node name before topic name
  ros::Publisher status_pub = this->node.advertise<xamla_sysmon_msgs::HeartBeat>("heartbeat", 100);
  ros::Rate loop_rate(this->freq);
  this->heartbeat_msg.status = this->status;
  this->heartbeat_msg.details = this->details;
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
  //TODO kill the thread here
}

int32_t xamla_sysmon_client::updateStatus(heartbeat_status new_status, std::string details)
{
  //TODO check if status is null or change it to type safe
  //ROS_INFO("update status called");
  this->status = new_status;
  this->details = details;
}

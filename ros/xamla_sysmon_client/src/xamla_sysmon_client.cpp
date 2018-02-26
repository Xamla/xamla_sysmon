#include "xamla_sysmon_client/xamla_sysmon_client.h"

xamla_sysmon_client::xamla_sysmon_client()
{
  this->is_start_called = false;
  this->stop_heartbeat = false;
}

void xamla_sysmon_client::start(ros::NodeHandle &node, unsigned int freq) {
  if (this->is_start_called == false)
    this->is_start_called = true;
  else //if (this->is_start_called == true)
  {
    ROS_WARN("Heartbeat: start already called once, nothing to do");
    return;
  }

  this->node= node;

  //checking for +ve frequency
  if(freq != 0)
    this->freq = freq;
  else
  {
    this->freq = 10;
    ROS_WARN("heatbeat freq cannot be 0");
  }
  ROS_INFO("[Heartbeat] publishing frequency: %d", this->freq);

  this->heartbeat_msg.header.stamp = ros::Time::now();
  this->heartbeat_msg.status = (int)TopicHeartbeatStatus::TopicCode::GO;
  this->heartbeat_msg.details = "Node starting";

  //openning new thread to publish state
  this->pub_thread = std::thread(&xamla_sysmon_client::publish_status, this);
}

void xamla_sysmon_client::publish_status()
{
  //ros publisher
  ROS_INFO("starting heartbeat publishing loop");
  ros::Publisher status_pub = this->node.advertise<xamla_sysmon_msgs::HeartBeat>(ros::this_node::getName()+"/heartbeat", 100);
  ros::Rate loop_rate(this->freq);

  while(ros::ok())
  {
    //if shutdown was called
    if (this->stop_heartbeat == true)
    {
      break;
    }

    {
      //thread lock guard
      std::mutex my_mutex;
      std::lock_guard<std::mutex> lock(my_mutex);
      status_pub.publish(this->heartbeat_msg);
    //ROS_INFO("published a heartbeat");
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  //shutting down
  //publisher gets out of scope and terminate no need to shut it down
  ROS_INFO("Heartbeat: stopping heartbeat");
}

void xamla_sysmon_client::shutdown()
{
  if (this->is_start_called == true)
    this->stop_heartbeat = true;
  else //(this->is_start_called == false)
    ROS_WARN("Heartbeat wasn't running");
}

void xamla_sysmon_client::updateStatus(TopicHeartbeatStatus::TopicCode new_status, std::string details)
{
  //lock guard
  std::mutex my_mutex;
  std::lock_guard<std::mutex> lock(my_mutex);
  //setting heartbeat msg fields
  this->heartbeat_msg.status = (int)new_status;
  this->heartbeat_msg.details = details;
}

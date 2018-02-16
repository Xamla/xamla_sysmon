#ifndef __XamlaSysmonWatch_H__
#define __XamlaSysmonWatch_H__

#include <string>
#include <iostream>
#include <thread>
#include <mutex>

#include <ros/duration.h>
#include <ros/ros.h>

#include <xamla_sysmon_msgs/HeartBeat.h>
#include <xamla_sysmon_msgs/SystemStatus.h>
#include <xamla_sysmon_msgs/statuscodes.h>

#define XAMLA_GLOBAL_STATE_TOPIC "/xamla_sysmon/system_status"

struct XamlaSysmonGlobalState
{
  bool go;
  bool nogo;
  bool only_secondary_error;
  std::string error_message;
  ros::Time time_stamp;
};

class XamlaSysmonWatch
{
public:
  XamlaSysmonWatch(int update_rate_in_hz_ = 10, double time_out_in_s = 0.2);
  ~XamlaSysmonWatch();

  XamlaSysmonGlobalState getGlobalStateSummary();
  xamla_sysmon_msgs::SystemStatus getGlobalState();
  void start();
  void shutdown();

private:
  void fetchGlobalStateThread();
  void handleGlobalStateMessageReceived(const xamla_sysmon_msgs::SystemStatus& message);

  bool received_global_state_;
  bool request_shutdown_;
  bool state_fetcher_running_;
  std::thread state_fetcher_;
  ros::Duration time_out_;
  int update_rate_in_hz_;
  xamla_sysmon_msgs::SystemStatus latest_global_state_;
  std::mutex mutex_;
};

#endif  // __XamlaSysmonWatch_H__

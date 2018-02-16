#include "xamla_sysmon_watch.h"
#include <ros/ros.h>
#include <thread>
#include <chrono>
#include <signal.h>

XamlaSysmonWatch xamla_sysmon_watch;

void signal_callback_handler(int signum)
{
  xamla_sysmon_watch.shutdown();
}

int main(int argc, char** argv)
{
  signal(SIGINT, signal_callback_handler);

  xamla_sysmon_watch.start();
  while (true)
  {
    XamlaSysmonGlobalState summary = xamla_sysmon_watch.getGlobalStateSummary();
    ROS_INFO("Global State is %s", summary.go == true ? "GO" : "NOGO");

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}
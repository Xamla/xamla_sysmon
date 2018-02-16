#include <cmath>
#include <string>
#include <functional>

#include <ros/ros.h>

#include <xamla_sysmon_node/monitor_base.h>

double XamlaSysmonMonitorBase::getTimeDiff(const ros::Time& last, const ros::Time& now, int precision)
{
  double scale = pow(10.0, precision);
  return round((last - now).toSec() * scale) / scale;
}
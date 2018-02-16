#include <string>
#include <stdio.h>

#include <xamla_sysmon_msgs/statuscodes.h>

extern "C" const char* generateMessageText(int status_code)
{
  // TODO: find a more elegant way to return this string
  static std::string msg = TopicHeartbeatStatus::generateMessageText(TopicHeartbeatStatus::intToStatusCode(status_code));
  return msg.c_str();
}

extern "C" int getStatusCode(const char* status_string)
{
  return TopicHeartbeatStatus::statusCodeToInt(TopicHeartbeatStatus::stringToStatusCode(status_string));
}

#ifndef __XamlaSysmonHeartbeat_H__
#define __XamlaSysmonHeartbeat_H__

#include <string>

class TopicHeartbeatStatus
{
public:
  // When adding a new status code to this list/changing values, check all functions in this class
  // if they need updates!
  enum class TopicCode
  {
    GO = 0,
    STARTING = 100,
    INTERNAL_ERROR = 200,
    EMERGENCY_STOP = 300,
    SECONDARY_ERROR = 400,
    INVALID = -1
  };

  enum class HeartbeatCode
  {
    ESTABLISHED = 0,
    TIMEOUT = 1
  };

  enum class SystemCode
  {
    GO = 0,
    ONLY_SECONDARY_ERROR = 1,
    ERROR = 2,
    EMERGENCY_STOP = 4,
    UNKOWN_ERROR = 8
  };

  TopicCode static intToStatusCode(int int_status_code);
  int static statusCodeToInt(TopicCode status_code);
  TopicCode static stringToStatusCode(const std::string& status_string);
  std::string static generateMessageText(TopicCode status_code);
  bool static checkStatusOK(TopicCode status_code);
  bool static checkIfNodeStatusCode(TopicCode status_code);
};

#endif

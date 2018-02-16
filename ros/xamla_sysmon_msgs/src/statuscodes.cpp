#include <string>
#include <iostream>
#include <xamla_sysmon_msgs/statuscodes.h>

TopicHeartbeatStatus::TopicCode TopicHeartbeatStatus::intToStatusCode(int int_status_code)
{
  switch (int_status_code)
  {
    case 0:
      return TopicCode::GO;
    case 100:
      return TopicCode::STARTING;
    case 200:
      return TopicCode::INTERNAL_ERROR;
    case 300:
      return TopicCode::EMERGENCY_STOP;
    case 400:
      return TopicCode::SECONDARY_ERROR;
    default:
    {
      std::cout << "[intToStatusCode] invalid status code: " << int_status_code << std::endl;
      return TopicCode::INVALID;
    }
  }
}

int TopicHeartbeatStatus::statusCodeToInt(TopicHeartbeatStatus::TopicCode status_code)
{
  return static_cast<int>(status_code);
}

TopicHeartbeatStatus::TopicCode TopicHeartbeatStatus::stringToStatusCode(const std::string& status_string)
{
  if (status_string == "GO")
    return TopicCode::GO;
  else if (status_string == "STARTING")
    return TopicCode::INTERNAL_ERROR;
  else if (status_string == "INTERNAL_ERROR")
    return TopicCode::INTERNAL_ERROR;
  else if (status_string == "EMERGENCY_STOP")
    return TopicCode::EMERGENCY_STOP;
  else if (status_string == "SECONDARY_ERROR")
    return TopicCode::SECONDARY_ERROR;
  else
  {
    std::cout << "[stringToStatusCode] invalid status code: " << status_string << std::endl;
    return TopicCode::INVALID;
  }
}

std::string TopicHeartbeatStatus::generateMessageText(TopicHeartbeatStatus::TopicCode status_code)
{
  switch (status_code)
  {
    case TopicCode::GO:
      return "Node is GO.";
    case TopicCode::STARTING:
      return "Node is starting up.";
    case TopicCode::INTERNAL_ERROR:
      return "Node has an internal error or timed out.";
    case TopicCode::EMERGENCY_STOP:
      return "Node is in emergency stop.";
    case TopicCode::SECONDARY_ERROR:
      return "This node is GO, but another node is faulty.";
    default:
      return "Unknown status code";
  }
}

bool TopicHeartbeatStatus::checkStatusOK(TopicHeartbeatStatus::TopicCode status_code)
{
  switch (status_code)
  {
    // List of all states indicating a Node/Topic is working as expected
    case TopicCode::GO:
      return true;
    default:
      return false;
  }
}

bool TopicHeartbeatStatus::checkIfNodeStatusCode(TopicHeartbeatStatus::TopicCode status_code)
{
  switch (status_code)
  {
    // List of all states indicating a Node/Topic is working as expected
    case TopicCode::GO:
      return true;
    default:
      return false;
  }
}

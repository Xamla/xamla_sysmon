/*
statuscodes.cpp

Copyright (c) 2018, Xamla and/or its affiliates.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Xamla and/or its affiliates nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL XAMLA AND/OR ITS AFFILIATES BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

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

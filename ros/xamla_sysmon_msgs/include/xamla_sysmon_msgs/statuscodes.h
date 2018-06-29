/*
statuscodes.h

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

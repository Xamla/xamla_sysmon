--[[
watch.lua

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
]]

local ros = require 'ros'
local xamla_sysmon = require 'xamla_sysmon.env'

local function test_flag(value, flag)
  return value % (2 * flag) >= flag
end

local Watch = torch.class('xamla_sysmon.Watch', xamla_sysmon)

function Watch:__init(node_handle, time_out_in_s)
  self.time_out = time_out_in_s
  self.subscriber = node_handle:subscribe("/xamla_sysmon/system_status", 'xamla_sysmon_msgs/SystemStatus', 1)
  self.subscriber:registerCallback(function(message, header)
    self:handleNewGlobalState(message, header)
  end)
  self.latest_global_state = nil
end

function Watch:handleNewGlobalState(message, header)
  self.latest_global_state = message
  if self.onNewGlobalState ~= nil then
    self.onNewGlobalState(self.latest_global_state)
  end
end

function Watch:getGlobalState()
  if self.latest_global_state == nil then
    local string_spec = ros.MsgSpec('xamla_sysmon_msgs/SystemState')
    local result = ros.Message(string_spec)
    result.header.stamp = ros.Time.now()
    result.system_status = 8
    return result
  else
    return self.latest_global_state
  end
end

function Watch:getGlobalStateSummary()
  local summary = {}
  summary.go = false
  summary.no_go = true
  summary.only_secondary_error = false
  summary.error_message = "Invalid State"
  summary.time_stamp = ros.Time(0)

  if self.latest_global_state == nil then
    summary.no_go = true
    summary.error_message = "XamlaSysmonWatch has not received any global state updates yet"
    summary.time_stamp = ros.Time.now()
  elseif (ros.Time.now() - self.latest_global_state.header.stamp):toSec() >= self.time_out then
    summary.no_go = true
    summary.error_message = "The state of XamlaSysmonWatch is outdated."
    summary.time_stamp = ros.Time.now()
  else
    if self.latest_global_state.system_status == 0 then
      summary.go = true
      summary.no_go = false
      summary.error_message = ""
      summary.time_stamp = self.latest_global_state.header.stamp
    elseif self.latest_global_state.system_status == 1 then
      summary.no_go = true
      summary.only_secondary_error = true
      summary.error_message = "There are only secondary erros."
      summary.time_stamp = self.latest_global_state.header.stamp
    else
      summary.no_go = true
      summary.time_stamp = self.latest_global_state.header.stamp
      summary.error_message = ""
      if test_flag(self.latest_global_state.system_status, 8) then
        summary.error_message = summary.error_message .. "At least one node is in INVALID state. "
      end
      if test_flag(self.latest_global_state.system_status, 4) then
        summary.error_message = summary.error_message .. "At least one node is in EMERGENCY_STOP state. "
      end
      if test_flag(self.latest_global_state.system_status, 2) then
        summary.error_message = summary.error_message .. "At least one node is in INTERNAL_ERROR state. "
      end
      if test_flag(self.latest_global_state.system_status, 1) then
        summary.error_message = summary.error_message .. "At least one node is in SECONDARY_ERROR state. "
      end
    end
  end

  return summary
end

function Watch:shutdown()
  self.subscriber:shutdown()
end

return Watch;

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

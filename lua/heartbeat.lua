local ros = require 'ros'
local xamla_sysmon = require 'xamla_sysmon.env'


local Heartbeat = torch.class('xamla_sysmon.Heartbeat', xamla_sysmon)

Heartbeat.GO = "GO"
Heartbeat.STARTING = "STARTING"
Heartbeat.SECONDARY_ERROR = "SECONDARY_ERROR"
Heartbeat.INTERNAL_ERROR = "INTERNAL_ERROR"
Heartbeat.EMERGENCY_STOP = "EMERGENCY_STOP"

Heartbeat.SYSTEM_STATE = {}
Heartbeat.SYSTEM_STATE.GO = 0
Heartbeat.SYSTEM_STATE.SECONDARY_ERROR = 1
Heartbeat.SYSTEM_STATE.ERROR = 2
Heartbeat.SYSTEM_STATE.EMERGENCY_STOP = 4
Heartbeat.SYSTEM_STATE.UNKOWN_ERROR = 8

function Heartbeat:__init()
  self.heartbeat_publisher = nil
  self.frequency = 10
  self.last_publish_time = ros.Time(0)

  self.heartbeat_spec = ros.MsgSpec('xamla_sysmon_msgs/HeartBeat')
  self.msg = ros.Message(self.heartbeat_spec)

  self.status = Heartbeat.STARTING
  self.last_publish_status = nil
  self.msg.status = xamla_sysmon.status:getStatusCode(Heartbeat.STARTING)
  self.msg.details = xamla_sysmon.status:generateMessageText(self.msg.status)
end

function Heartbeat:start(node_handle, frequency)

  self.heartbeat_publisher = node_handle:advertise('heartbeat',  self.heartbeat_spec, 10, false)
  self.frequency = frequency
  self.last_publish_time = ros.Time.now()
  ros.INFO("[Heartbeat] publishing frequency: ".. self.frequency )
  --ros.INFO(tostring(self.msg))
end

function Heartbeat:publish()
  if self.heartbeat_publisher == nil then
    ros.ERROR("Cannot publish on an uninitialized/stopped heartbeat topic: \n" .. debug.traceback())
    return
  end

  if (ros.Time.now() - self.last_publish_time):toSec() >= 1.0/self.frequency or self.last_publish_status ~= self.status then
    if self.heartbeat_publisher ~= nil then
      self.msg.header.stamp = ros.Time.now()
      self.heartbeat_publisher:publish(self.msg)
      self.last_publish_time = ros.Time.now()
      self.last_publish_status = self.status
    end
  end
end

function Heartbeat:updateStatus(new_status, details)
  if(new_status == nil) then
    ros.ERROR("Invalid new_status: \n" .. debug.traceback())
    self.status = Heartbeat.INTERNAL_ERROR
    self.msg.status = xamla_sysmon.status:getStatusCode(Heartbeat.INTERNAL_ERROR)
    self.msg.details = "Invalid internal node state"
  else
    self.status = new_status
    self.msg.status = xamla_sysmon.status:getStatusCode(new_status)
    self.msg.details = details
  end

  self:publish()
end

function Heartbeat:getStatus()
  return self.status
end

return Heartbeat

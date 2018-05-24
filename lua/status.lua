local torch = require 'torch'

local xamla_sysmon = require 'xamla_sysmon.env'


local Statuscodes = torch.class('xamla_sysmon.status', xamla_sysmon)

local function setDefault (t, d)
  local mt = {__index = function () return d end}
  setmetatable(t, mt)
end

local SysmonStatusCodes = {
 GO = 0,
 STARTING = 100,
 INTERNAL_ERROR = 200,
 EMERGENCY_STOP = 300,
 SECONDARY_ERROR = 400,
 [0] = "GO",
 [100] = "STARTING",
 [200] = "INTERNAL_ERROR",
 [300] = "EMERGENCY_STOP",
 [400] = "SECONDARY_ERROR"
}

local StatusCodesToDetails = {
 GO = "Node is GO.",
 STARTING = "Node is starting up.",
 INTERNAL_ERROR = "Node has an internal error or timed out.",
 EMERGENCY_STOP = "Node is in emergency stop.",
 SECONDARY_ERROR = "This node is GO, but another node is faulty."
}

setDefault (SysmonStatusCodes, "INVALID")
setDefault (StatusCodesToDetails, "Unknown status code")

function Statuscodes:__init()

end

function Statuscodes:generateMessageText(status_code)
  return StatusCodesToDetails[SysmonStatusCodes[status_code]]
end

function Statuscodes:getStatusCode(status_string)
  return SysmonStatusCodes[status_string]
end

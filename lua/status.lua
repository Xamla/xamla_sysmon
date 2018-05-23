local torch = require 'torch'

local xamla_sysmon = require 'xamla_sysmon.env'


local Statuscodes = torch.class('xamla_sysmon.status', xamla_sysmon)

local StringToInt = {}
StringToInt["GO"] = 0
StringToInt["STARTING"] = 100
StringToInt["INTERNAL_ERROR"] = 200
StringToInt["EMERGENCY_STOP"] = 300
StringToInt["SECONDARY_ERROR"] = 400

local IntToDetails = {}
IntToDetails["0"] = "Node is GO."
IntToDetails["100"] = "Node is starting up."
IntToDetails["200"] = "Node has an internal error or timed out."
IntToDetails["300"] = "Node is in emergency stop."
IntToDetails["400"] = "This node is GO, but another node is faulty."

function Statuscodes:__init()

setDefault (StringToInt, -1)
setDefault (IntToDetails, "Unknown status code")

end

local function setDefault (t, d)
  local mt = {__index = function () return d end}
  setmetatable(t, mt)
end

function Statuscodes:generateMessageText(status_code)
  return IntToDetails[tostring(status_code)]
end

function Statuscodes:getStatusCode(status_string)
  return StringToInt[status_string]
end

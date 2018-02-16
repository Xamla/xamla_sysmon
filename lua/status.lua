local ffi = require 'ffi'
local torch = require 'torch'

local xamla_sysmon = require 'xamla_sysmon.env'


local Statuscodes = torch.class('xamla_sysmon.status', xamla_sysmon)

function Statuscodes:__init()
end

function Statuscodes:generateMessageText(status_code)
  return ffi.string(xamla_sysmon.lib.generateMessageText(status_code))
end

function Statuscodes:getStatusCode(status_string)
  return xamla_sysmon.lib.getStatusCode(status_string)
end

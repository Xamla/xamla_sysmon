--[[
status.lua

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

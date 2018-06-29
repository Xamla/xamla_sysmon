--[[
demo_server.lua

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
local xamal_sysmon = require 'xamla_sysmon'

ros.init('demo_monitor')
local nh = ros.NodeHandle("~")


local function testBit(val, bit_pos)
  local function genBit(bit_pos)
    return 2 ^ (bit_pos - 1)  -- 1-based indexing
  end

  return val % (2*genBit(bit_pos)) >= genBit(bit_pos)
end


function newMessage(msg, header)
  local status_bit = {}
  status_bit[1] = "At least one node reports an error"
  status_bit[2] = "At least one node is unconfigured"
  status_bit[3] = "At least one node is not reachable"
  status_bit[4] = "System Emergency Stop"
  status_bit[5] = "Invalid"

  local line_count = 30   -- utility variable to get fixed number of lines in print

  print("Last update at " .. msg.header.stamp)
  print("System Status:")
  line_count = line_count - 1
  local no_errors = true
  for i=1,4 do
    if testBit(msg.system_status, i) then
      print("    " .. status_bit[i])
      line_count = line_count - 1
      no_errors = false
    end
  end
  if no_errors then
    print("System is ready to go")
    line_count = line_count - 1
  end

  print("Per node/topic report:")
  line_count = line_count - 1
  for i=1,#msg.topics do
    print("  topic \"" .. msg.topics[i] .. "\" reports " ..   xamal_sysmon.status():generateMessageText(msg.err_code[i])
          .. " (msg: " .. msg.topic_msg[i] .. ")")
    line_count = line_count - 1
  end

  for i=line_count,1,-1 do
    print("")
  end
end


function main()
-- subscribe to dummy_chat topic with no messages back-log
-- transport_options (arguments 4 & 5) are optional - used here only for demonstration purposes
  subscriber = nh:subscribe("/xamla_sysmon/system_status", 'xamla_sysmon_msgs/SystemStatus', 1, { 'udp', 'tcp' }, { tcp_nodelay = true })
  subscriber:registerCallback(newMessage)

  while ros.ok() do
    ros.spinOnce()
    sys.sleep(0.1)
  end

end

main()
ros.shutdown()

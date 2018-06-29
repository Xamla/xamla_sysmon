--[[
demo_node.lua

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

ros = require 'ros'
xamal_sysmon = require 'xamla_sysmon'

ros.init('demo_node')
nh = ros.NodeHandle("~")


function connectCb(name, topic)
  print("subscriber connected: " .. name .. " (topic: '" .. topic .. "')")
end

function disconnectCb(name, topic)
  print("subscriber diconnected: " .. name .. " (topic: '" .. topic .. "')")
end



function main()
  string_spec = ros.MsgSpec('std_msgs/String')
  heartbeat = xamal_sysmon.Heartbeat()
  heartbeat:start(nh, 0.5)

  publisher = nh:advertise("some_data", string_spec, 1, false, connectCb, disconnectCb)
  ros.spinOnce()

  local msg = ros.Message(string_spec)
  local count=0
  while ros.ok() do
    if publisher:getNumSubscribers() > 0 then
      msg.data = "Hello World"
      publisher:publish(msg)
    end

    heartbeat:publish()

    if count == 2 then
      heartbeat:updateStatus(heartbeat.STARTING, "starting ...")
    elseif count == 6 then
      heartbeat:updateStatus(heartbeat.GO, "")
    elseif count == 10 then
      heartbeat:updateStatus(heartbeat.INTERNAL_ERROR, "Test Error")
    elseif count > 15 then
      count=0
    end

    sys.sleep(0.5)
    ros.spinOnce()
    count = count+1
  end
end

main()
ros.shutdown()

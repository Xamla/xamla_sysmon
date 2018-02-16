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

# Xamla ROS Sysmon

Xamla ROS Sysmon monitors ROS topics and publishes a global system state depending of the state of the topics. For regular ROS topics it monitors the update frequency of the topic and will go into a fail state whenever a configurable threshold is reached. For a finer control of system state it is recommended to make use if Xamla ROS Heartbeat, which is a convention on top of regular ROS topics.

## Xamla ROS Heartbeat
To use Xamla ROS Heartbeat a node shall publish a topic called 'heartbeat' within its namespace. It is within the responsibility of the node to update the heartbeat with the frequency configured in Xamla ROS Sysmon. If the timeout threshold is reached the node is considered as faulty and the global system state will go into fail state. The following states are available for heartbeats:

|State | Explanation |
|---|---|
|STARTING|A node might take a while from being started until reaching a state where it can be seen as ready. During this time the node shall take the state STARTING|
|GO | A node shall take the GO state when everything is working as expected |
|INTERNAL\_ERROR| Whenever a node detects an error within its own responsibility it shall go into INTERNAL\_ERROR state|
|EMERGENCY\_STOP| Whenever a node detects the usage of the emergency stop button it shall go into EMERGENCY\_STOP state.|
|SECONDARY\_ERROR| A node shall go into SECONDARY\_ERROR whenever it detects an error in a node it is depending on and everything within its own responsibility is working as expected. This is reflected in the global system state in order to give nodes the possibilty to recover when the dependency has recovered|

## Xamla Global State Watch
Xamla ROS Sysmon includes an easy to use helper class, which interpretes the Global State and returns either GO or NOGO. It includes a configurable timeout to make sure that GO is only returned when the Global State is up to date. Also it contains a flag for indicating that only secondary errors exist in order to give nodes the possibility to revover. **Note:** When using global state watch make sure that there is no other subscriber to `/xamla_sysmon/system_status` in the used node_handle, otherwise the watcher might not recieve the global state correctly.

### Functions
|Name|Description|
|---|---|
|__Init(node\_handle, time\_out\_in\_s)|node\_handle: A torch-ros node\_handle object <br/> time\_out\_in\_s: Maximum age of Global State ROS message to be considered as valid|
|getGlobalState()|Returns the latest Global State ROS message|
|getGlobalStateSummary()|Returns the Global State summary object, see below|

### Global State Summary
|Property|Description|
|---|---|
|go|Boolean: True when global state is GO|
|no_go|Boolean: True when global state is not GO or when the age of the latest Global State is bigger than the time\_out|
|only\_secondary\_error|Boolean: True when global state is SECONDARY_ERROR.|
|error\_message|String: A description of the current error|
|time_stamp|ROS.Time: Timestamp from the latest global state ros message |

### Usage pseudo code example
```
ros = require 'ros'
xamla_sysmon = require 'xamla_sysmon'

ros.init('test')
local nh = ros.NodeHandle("~")
local sysmon_watch = xamla_sysmon.Watch.new(nh, 0.1)

while ros.ok()
    local sys_state = sysmon_watch:getGlobalStateSummary()
    if sys_state.go == true then
        -- do something that should only be done when all sub systems are GO
    else
        -- wait for recovery from error
    end

    ros.spinOnce()
    sys.sleep(0.1)
end
```

## Global State
The global system is published on `/xamla_sysmon/system_status` and is an aggregation of the states of the monitored nodes. It is considered as bit array, where the four least significant bits encode the error state. The following states are available:

|State| Explanation|
|---|---|
|GO <br/> no bit set (0)|All monitored topics are in GO state.|
|SECONDARY\_ERROR <br /> first bit (1)|At least one monitored topic is in SECONDARY\_ERROR state. |
|ERROR <br /> second bit (2)|At least on monitored topic is either in INTERNAL_ERROR state or the latest update is to long ago (timed out)|
|EMERGENCY\_STOP <br /> third bit (4) | At least on monitored topic is in EMERGENCY_STOP state.|
|UNKOWN\_ERROR <br /> fourth bit (8)| At least on monitored topic is an unkown state.|

It is possible that the global state encodes several of the above states. Consider the example where one node goes in ERROR state and a dependend node goes into SECONDARY_ERROR state. In this case the global state would have the second and first bit set, which would be 3 interpreted as integer.

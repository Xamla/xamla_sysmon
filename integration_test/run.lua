--[[
run.lua

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
xamla_sysmon = require 'xamla_sysmon'

ros.init('integration_test')

local spinner = ros.AsyncSpinner()
spinner:start()
local nh = ros.NodeHandle("~")

local heartbeat_spec = 'std_msgs/String'
local topic_spec = 'std_msgs/String'
local sysmon_spec = 'xamla_sysmon_msgs/SystemStatus'
local topicPublishers = {}

local heartbeat = xamla_sysmon.Heartbeat.new()
local test_state = {}
test_state.GO = heartbeat.GO
test_state.TIMEOUT = 1
test_state.ERROR = heartbeat.INTERNAL_ERROR
test_state.STARTING = heartbeat.STARTING
test_state.SECONDARY_ERROR = heartbeat.SECONDARY_ERROR
test_state.EMERGENCY_STOP = heartbeat.EMERGENCY_STOP

local test_cases = {}
local SLEEP = 0.1
local SYSMON_FREQUENCY = 2
local TOPIC_TIMEOUT = 4 + SYSMON_FREQUENCY

local heartbeat_topics = {
    ['topic1'] = test_state.GO,
    ['topic1err'] = ''
}

local default_topics = {
    ['default1'] = test_state.GO,
    ['default2'] = test_state.GO
}

local latest_system_state = nil

local sysmon_watch = xamla_sysmon.Watch.new(nh, SYSMON_FREQUENCY + 0.1)

function handleNewGlobalState(new_global_state)
    latest_system_state = new_global_state
end

function advertiseTopics()
    heartbeat:start(nh, 0.5) --[Hz]
    heartbeat:updateStatus(heartbeat.STARTING, 'Init ...')
    heartbeat:publish()

    local spec = ros.MsgSpec(topic_spec)
    for k, v in pairs(default_topics) do
        topicPublishers[k] = nh:advertise(k, spec, 10, false)
    end
end

function publishOnTopics()
    local spec = ros.MsgSpec(heartbeat_spec)
    if (heartbeat_topics['topic1'] ~= test_state.TIMEOUT) then
        heartbeat:updateStatus(heartbeat_topics['topic1'], heartbeat_topics['topic1err'])
        heartbeat:publish()
    end

    spec = ros.MsgSpec(topic_spec)
    for k, v in pairs(default_topics) do
        if (v ~= test_state.TIMEOUT) then
            local msg = ros.Message(topic_spec)
            topicPublishers[k]:publish(msg)
        end
    end
end

function runTest(name)
    print("runTest", name)
    local co = coroutine.create(test_cases[name])
    local test_passed = true
    local state, result, is, should
    while (coroutine.status(co) ~= 'dead') do
        state, result, is, should = coroutine.resume(co)
        test_passed = test_passed and result
        publishOnTopics()
        ros.spinOnce()
        sys.sleep(SLEEP)
    end
    local s = 'FAILED'
    if test_passed == true then
        s = 'passed'
    end
    print(s, 'is', is, 'expected', should)

    return s
end

function expect(is, should)
    return is == should, is, should
end

test_cases.names = {
    'SystemStateShouldBeGo_When_AllWatchedTopicsAreGo',
    'SystemStateShouldBeError_When_AtLeastOneWatchedTopicIsError',
    'SystemStateShouldBeError_When_AtLeastOneWatchedTopicIsStarting',
    'SystemStateShouldBeEmergencyStop_When_AtLeastOneWatchedTopicIsEmergencyStop',
    'SystemStateShouldBeSecondaryError_When_AllWatchedTopicAreEitherGoOrSecondaryError',
    'SystemStateShouldBeError_When_DefaultTopicTimesOut',
    'SystemStateShouldBeGo_When_AllWatchedTopicsAreGo',
    'SystemStateShouldBeError_When_HeartbeatTopicTimesOut',
    'SystemStateShouldRecoverFromError_When_AllWatchedNodesDo',

    'SystemStateSummaryShouldBeGo_When_SystemStateIsGo',
    'SystemStateSummaryShouldBeNoGo_When_SystemStateIsNotGo',
    'SystemStateSummaryShouldBeNoGoAndSecondaryError_When_SystemStateIsSecondaryError',
    'SystemStateSummaryShouldBeNoGo_When_SystemStateTimestampIsTooOld'
}

test_cases.SystemStateShouldBeGo_When_AllWatchedTopicsAreGo = function()
    heartbeat_topics['topic1'] = test_state.GO
    heartbeat_topics['topic1err'] = 'Go go go'
    default_topics['default1'] = test_state.GO
    default_topics['default2'] = test_state.GO

    local cnt = 0
    while cnt * SLEEP <= SYSMON_FREQUENCY + 0.1 do
        cnt = cnt + 1
        coroutine.yield(true) -- give control back to test loop for publishing messages on topics
    end

    return expect(latest_system_state.system_status, heartbeat.SYSTEM_STATE.GO)
end

test_cases.SystemStateShouldBeError_When_AtLeastOneWatchedTopicIsError = function()
    heartbeat_topics['topic1'] = test_state.ERROR
    heartbeat_topics['topic1err'] = 'Something broke'
    default_topics['default1'] = test_state.GO
    default_topics['default2'] = test_state.GO
    local cnt = 0
    while cnt * SLEEP <= SYSMON_FREQUENCY + 0.1 do
        cnt = cnt + 1
        coroutine.yield(true) -- give control back to test loop for publishing messages on topics
    end

    return expect(latest_system_state.system_status, heartbeat.SYSTEM_STATE.ERROR)
end

test_cases.SystemStateShouldBeError_When_AtLeastOneWatchedTopicIsStarting = function()
    heartbeat_topics['topic1'] = test_state.STARTING
    heartbeat_topics['topic1err'] = 'Booting up'
    default_topics['default1'] = test_state.GO
    default_topics['default2'] = test_state.GO

    local cnt = 0
    while cnt * SLEEP <= SYSMON_FREQUENCY + 0.1 do
        cnt = cnt + 1
        coroutine.yield(true) -- give control back to test loop for publishing messages on topics
    end

    return expect(latest_system_state.system_status, heartbeat.SYSTEM_STATE.ERROR)
end

test_cases.SystemStateShouldBeEmergencyStop_When_AtLeastOneWatchedTopicIsEmergencyStop = function()
    heartbeat_topics['topic1'] = test_state.EMERGENCY_STOP
    heartbeat_topics['topic1err'] = 'ALERT!'
    default_topics['default1'] = test_state.GO
    default_topics['default2'] = test_state.GO

    local cnt = 0
    while cnt * SLEEP <= SYSMON_FREQUENCY + 0.1 do
        cnt = cnt + 1
        coroutine.yield(true) -- give control back to test loop for publishing messages on topics
    end

    return expect(latest_system_state.system_status, heartbeat.SYSTEM_STATE.EMERGENCY_STOP)
end

test_cases.SystemStateShouldBeSecondaryError_When_AllWatchedTopicAreEitherGoOrSecondaryError = function()
    heartbeat_topics['topic1'] = test_state.SECONDARY_ERROR
    heartbeat_topics['topic1err'] = '2nd'
    default_topics['default1'] = test_state.GO
    default_topics['default2'] = test_state.GO

    local cnt = 0
    while cnt * SLEEP <= SYSMON_FREQUENCY + 0.1 do
        cnt = cnt + 1
        coroutine.yield(true) -- give control back to test loop for publishing messages on topics
    end

    return expect(latest_system_state.system_status, heartbeat.SYSTEM_STATE.SECONDARY_ERROR)
end

test_cases.SystemStateShouldBeError_When_DefaultTopicTimesOut = function()
    heartbeat_topics['topic1'] = test_state.GO
    heartbeat_topics['topic1err'] = ''
    default_topics['default1'] = test_state.TIMEOUT
    default_topics['default2'] = test_state.GO

    local cnt = 0
    while cnt * SLEEP <= TOPIC_TIMEOUT + 0.1 do
        cnt = cnt + 1
        coroutine.yield(true) -- give control back to test loop for publishing messages on topics
    end

    return expect(latest_system_state.system_status, heartbeat.SYSTEM_STATE.ERROR)
end

test_cases.SystemStateShouldBeError_When_HeartbeatTopicTimesOut = function()
    heartbeat_topics['topic1'] = test_state.TIMEOUT
    heartbeat_topics['topic1err'] = 'TIMEOUT'
    default_topics['default1'] = test_state.GO
    default_topics['default2'] = test_state.GO

    local cnt = 0
    while cnt * SLEEP <= TOPIC_TIMEOUT + 0.1 do
        cnt = cnt + 1
        coroutine.yield(true) -- give control back to test loop for publishing messages on topics
    end

    return expect(latest_system_state.system_status, heartbeat.SYSTEM_STATE.ERROR)
end

test_cases.SystemStateShouldRecoverFromError_When_AllWatchedNodesDo = function()
    heartbeat_topics['topic1'] = test_state.GO
    heartbeat_topics['topic1err'] = 'Long test is long'
    default_topics['default1'] = test_state.TIMEOUT
    default_topics['default2'] = test_state.GO

    local cnt = 0
    while cnt * SLEEP <= TOPIC_TIMEOUT + 0.1 do
        cnt = cnt + 1
        coroutine.yield(true) -- give control back to test loop for publishing messages on topics
    end

    local result, is, should = expect(latest_system_state.system_status, heartbeat.SYSTEM_STATE.ERROR)
    if (result == false) then
        print("Did not go into Error")
        return result, is, should
    end

    heartbeat_topics['topic1'] = test_state.SECONDARY_ERROR
    default_topics['default1'] = test_state.TIMEOUT
    default_topics['default2'] = test_state.GO

    cnt = 0
    while cnt * SLEEP <= SYSMON_FREQUENCY + 0.1 do
        cnt = cnt + 1
        coroutine.yield(true) -- give control back to test loop for publishing messages on topics
    end

    result, is, should = expect(latest_system_state.system_status, heartbeat.SYSTEM_STATE.ERROR + heartbeat.SYSTEM_STATE.SECONDARY_ERROR)
    if (result == false) then
        print("Did not go into Error + SecondaryError")
        return result, is, should
    end

    heartbeat_topics['topic1'] = test_state.SECONDARY_ERROR
    default_topics['default1'] = test_state.GO
    default_topics['default2'] = test_state.GO

    cnt = 0
    while cnt * SLEEP <= SYSMON_FREQUENCY + 0.1 do
        cnt = cnt + 1
        coroutine.yield(true) -- give control back to test loop for publishing messages on topics
    end

    result, is, should = expect(latest_system_state.system_status, heartbeat.SYSTEM_STATE.SECONDARY_ERROR)
    if (result == false) then
        print("Did not go back to SecondaryError")
        return result, is, should
    end

    heartbeat_topics['topic1'] = test_state.GO
    default_topics['default1'] = test_state.GO
    default_topics['default2'] = test_state.GO

    cnt = 0
    while cnt * SLEEP <= SYSMON_FREQUENCY + 0.1 do
        cnt = cnt + 1
        coroutine.yield(true) -- give control back to test loop for publishing messages on topics
    end

    result, is, should = expect(latest_system_state.system_status, heartbeat.SYSTEM_STATE.GO)
    return result, is, should
end

function test_cases.SystemStateSummaryShouldBeGo_When_SystemStateIsGo()
  heartbeat_topics['topic1'] = test_state.GO
  default_topics['default1'] = test_state.GO
  default_topics['default2'] = test_state.GO

  local cnt = 0
  while cnt * SLEEP <= SYSMON_FREQUENCY + 0.1 do
    cnt = cnt + 1
    coroutine.yield(true) -- give control back to test loop for publishing messages on topics
  end

  sys_state = sysmon_watch:getGlobalStateSummary()
  local result1, is1, should1 = expect(sys_state.go, true)
  print("go", result1, is1, should1)
  local result2, is2, should2 = expect(sys_state.no_go, false)
  print("nogo", result2, is2, should2)
  local result3, is3, should3 = expect(sys_state.only_secondary_error, false)
  print("2nd", result3, is3, should3)

  return result1 and result2 and result3
end

function test_cases.SystemStateSummaryShouldBeNoGo_When_SystemStateIsNotGo()
  heartbeat_topics['topic1'] = test_state.ERROR
  default_topics['default1'] = test_state.GO
  default_topics['default2'] = test_state.GO

  local cnt = 0
  while cnt * SLEEP <= SYSMON_FREQUENCY + 0.1 do
    cnt = cnt + 1
    coroutine.yield(true) -- give control back to test loop for publishing messages on topics
  end

  sys_state = sysmon_watch:getGlobalStateSummary()
  local result1, is1, should1 = expect(sys_state.go, false)
  print("go", result1, is1, should1)
  local result2, is2, should2 = expect(sys_state.no_go, true)
  print("nogo", result2, is2, should2)
  local result3, is3, should3 = expect(sys_state.only_secondary_error, false)
  print("2nd", result3, is3, should3)

  return result1 and result2 and result3
end

function test_cases.SystemStateSummaryShouldBeNoGoAndSecondaryError_When_SystemStateIsSecondaryError()
  heartbeat_topics['topic1'] = test_state.SECONDARY_ERROR
  default_topics['default1'] = test_state.GO
  default_topics['default2'] = test_state.GO

  local cnt = 0
  while cnt * SLEEP <= SYSMON_FREQUENCY + 0.1 do
    cnt = cnt + 1
    coroutine.yield(true) -- give control back to test loop for publishing messages on topics
  end

  sys_state = sysmon_watch:getGlobalStateSummary()
  local result1, is1, should1 = expect(sys_state.go, false)
  print("go", result1, is1, should1)
  local result2, is2, should2 = expect(sys_state.no_go, true)
  print("nogo", result2, is2, should2)
  local result3, is3, should3 = expect(sys_state.only_secondary_error, true)
  print("2nd", result3, is3, should3)

  return result1 and result2 and result3
end

function test_cases.SystemStateSummaryShouldBeNoGo_When_SystemStateTimestampIsTooOld()
  heartbeat_topics['topic1'] = test_state.GO
  default_topics['default1'] = test_state.GO
  default_topics['default2'] = test_state.GO

  local cnt = 0
  while cnt * SLEEP <= SYSMON_FREQUENCY + 0.1 do
    cnt = cnt + 1
    coroutine.yield(true) -- give control back to test loop for publishing messages on topics
  end

  sys_state = sysmon_watch:getGlobalStateSummary()
  local result, is, should = expect(sys_state.go, true)
  print("go", result, is, should)

  sys.sleep(SYSMON_FREQUENCY + 0.5)
  sys_state = sysmon_watch:getGlobalStateSummary()
  local result1, is1, should1 = expect(sys_state.go, false)
  print("go", result1, is1, should1)
  local result2, is2, should2 = expect(sys_state.no_go, true)
  print("nogo", result2, is2, should2)
  local result3, is3, should3 = expect(sys_state.only_secondary_error, false)
  print("2nd", result3, is3, should3)

  return result and result1 and result2 and result3
end

function main()
  advertiseTopics()
  --subscriber = nh:subscribe('/xamla_sysmon/system_status', sysmon_spec, 1)
  --subscriber:registerCallback(handleNewSysmonMessage)
  sysmon_watch.onNewGlobalState = handleNewGlobalState

  local getNextTestName = coroutine.create(function()
    for k, v in ipairs(test_cases.names) do
        coroutine.yield(v)
    end
  end)

  -- Publish a few messages on topics to initial system state
  for i = 1, 10 do
    publishOnTopics()
    ros.spinOnce()
    sys.sleep(0.1)
  end
  sys.sleep(SYSMON_FREQUENCY - 1)

  local testResults = {}
  local dt = ros.Rate(10)
  while ros.ok() --[[and coroutine.status(getNextTestName) ~= 'dead']] do
    if coroutine.status(getNextTestName) ~= 'dead' then
        local state, testName = coroutine.resume(getNextTestName)
        if (testName ~= nil) then
            testResults[testName] = runTest(testName)
        end
    else
        break
    end
    publishOnTopics()
    ros.spinOnce()
    dt:sleep()
  end

  print("")
  print(" ------ SUMMARY ------ ")
  local summary = {['FAILED'] = 0, ['passed'] = 0}
  local total = 0
  for k, v in pairs(testResults) do
    total = total + 1
    summary[v] = summary[v] + 1
  end
  print("#tests", total)
  print("passed: ", summary['passed'])
  print("failed: ", summary['FAILED'])
end

main()

spinner:stop()
ros.shutdown()
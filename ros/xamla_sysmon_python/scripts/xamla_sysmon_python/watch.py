#!/usr/bin/env python

'''
watch.py

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
'''

import rospy
from enum import IntEnum
from threading import Lock
from xamla_sysmon_msgs.msg import SystemStatus
import thread

_xamla_global_state_topic = "/xamla_sysmon/system_status"

# class SystemCode(IntEnum):
#     GO = 0
#     SECONDARY_ERROR = 1
#     ERROR = 2
#     EMERGENCY_STOP = 4
#     UNKOWN_ERROR = 8

class XamlaSysmonGlobalState():
    def __init__(self):
        self.go = None
        self.nogo = None
        self.only_secondary_error = None
        self.error_message = None
        self.time_stamp = None

class WatchClient():
    def __init__(self, rate, timeout):
        self.rate = rospy.Rate(rate)
        self.timeout = timeout

        self.state_fetcher_running = False
        self.mutex = Lock()
        self.received_global_state = False
        self.latest_global_state = None
        self.request_shutdown = False

    def __del__(self):
        self.shutdown()
        self.fetch_thread.join()

    def get_global_state(self):
        return self.latest_global_state

    def get_global_state_summary(self):
        global_state = XamlaSysmonGlobalState()
        global_state.go = False
        global_state.nogo = True
        global_state.only_secondary_error = False
        global_state.error_message = "Invalid State"
        global_state.time_stamp = rospy.Time.now()

        self.mutex.acquire()
        if not self.state_fetcher_running:
            global_state.nogo = True
            global_state.error_message = "The state fetcher thread is not running."
        elif not self.received_global_state:
            global_state.nogo = True
            global_state.error_message = "XamlaSysmonWatch has not received any global state updates yet"
        elif rospy.Time.now().to_sec() - float(self.latest_global_state.header.stamp.to_sec()) >= self.timeout:
            global_state.nogo = True
            global_state.error_message = "The state of XamlaSysmonWatch is outdated."
            global_state.time_stamp = rospy.Time.now()
        else:
            if self.latest_global_state.system_status == 0:
                global_state.go = True
                global_state.nogo = False
                global_state.error_message = ""
                global_state.time_stamp = self.latest_global_state.header.stamp
            elif self.latest_global_state.system_status == 1:
                global_state.nogo = True
                global_state.only_secondary_error = True
                global_state.error_message = "There are only secondary erros."
                global_state.time_stamp = self.latest_global_state.header.stamp
            else:
                global_state.nogo = True
                global_state.time_stamp = self.latest_global_state.header.stamp
                if self.latest_global_state.system_status & (1 << 3):
                    global_state.error_message = "At least one node is in INVALID state. "
                if self.latest_global_state.system_status & (1 << 2):
                    global_state.error_message = "At least one node is in EMERGENCY_STOP state. "
                if self.latest_global_state.system_status & (1 << 1):
                    global_state.error_message = "At least one node is in INTERNAL_ERROR state. "
                if self.latest_global_state.system_status & (1 << 0):
                    global_state.error_message = "At least one node is in SECONDARY_ERROR state. "
        self.mutex.release()
        return global_state

    def start(self):
        if not self.state_fetcher_running:
            self.fetch_thread = thread.start_new_thread(self.fetch_global_state_thread, ())

    def shutdown(self):
        self.mutex.acquire()
        self.request_shutdown = True
        self.mutex.release()

    def fetch_global_state_thread(self):
        self.state_fetcher_running = True
        rospy.Subscriber(_xamla_global_state_topic, SystemStatus, self.handle_global_state_message_received)
        rospy.loginfo("Subscribed to %s", _xamla_global_state_topic)

        rospy.spin()

        if not self.request_shutdown:
            rospy.logfatal("Fetch thread has exited ungracefully.")

        rospy.signal_shutdown("")

        self.state_fetcher_running = False

    def handle_global_state_message_received(self, msg):
        self.mutex.acquire()
        self.received_global_state = True
        self.latest_global_state = msg
        self.mutex.release()
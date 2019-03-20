#!/usr/bin/env python

'''
heartbeat.py

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
from xamla_sysmon_msgs.msg import HeartBeat
from enum import IntEnum
from threading import Lock
import thread
import math

class HeartbeatCode(IntEnum):
    GO = 0
    STARTING = 100
    INTERNAL_ERROR = 200
    EMERGENCY_STOP = 300
    SECONDARY_ERROR = 400
    INVALID = -1

class HeartbeatClient:
    def __init__(self):
        self.heartbeat_publisher = None
        self.stop_heartbeat = False
        self.mutex = Lock()
        self.msg = HeartBeat()

    def start(self, frequency, timeout=None):
        self.frequency = float(frequency)
        self.rate = rospy.Rate(frequency)
        if timeout is None:
            self.timeout = 4.0/self.frequency
            self.timeout = max(1.0, self.timeout)
        else:  
            self.timeout = float(timeout)
        self.heartbeat_publisher = rospy.Publisher(rospy.get_name() + '/heartbeat', HeartBeat, queue_size=10)
        self.msg.status = HeartbeatCode.STARTING
        self.msg.details = "STARTING"
        self.msg.header.stamp = rospy.Time.now()
        thread.start_new_thread(self.pub_thread, ())
        rospy.loginfo("[Heartbeat] publishing frequency: %f", self.frequency )

    def update_status(self, new_status, details):
        if self.heartbeat_publisher == None:
            rospy.logerr("Cannot publish on an uninitialized/stopped heartbeat topic")
            return
        if new_status != None:
            self.mutex.acquire()
            self.msg.status = new_status
            self.msg.details = details
            self.msg.header.stamp = rospy.Time.now()
            self.mutex.release()
        else:
            self.mutex.acquire()
            self.msg.status = HeartbeatCode.INVALID
            self.msg.details = "Invalid internal node state"
            self.msg.header.stamp = rospy.Time.now()
            self.mutex.release()

    def pub_thread(self):
        while not rospy.is_shutdown() and self.stop_heartbeat != True:
            self.mutex.acquire()
            if rospy.Time.now().to_sec() - self.msg.header.stamp.to_sec() >= self.timeout:
                self.msg.status = HeartbeatCode.INTERNAL_ERROR
                self.msg.details = "Watchdog: the duration between two updates was too long, error"
                self.msg.header.stamp = rospy.Time.now()
            self.heartbeat_publisher.publish(self.msg)
            self.mutex.release()
            self.rate.sleep()

    def shutdown(self):
        self.stop_heartbeat = True


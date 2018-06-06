#!/usr/bin/env python
import rospy
from xamla_sysmon_msgs.msg import HeartBeat
from enum import IntEnum
from threading import Lock
import thread

class HeartbeatCode(IntEnum):
    GO = 0
    STARTING = 100
    INTERNAL_ERROR = 200
    EMERGENCY_STOP = 300
    SECONDARY_ERROR = 400
    INVALID = -1

class SystemCode(IntEnum):
    GO = 0
    SECONDARY_ERROR = 1
    ERROR = 2
    EMERGENCY_STOP = 4
    UNKOWN_ERROR = 8

class HeartbeatClient:
    def __init__(self):
        self.heartbeat_publisher = None
        self.frequency = 10
        self.stop_heartbeat = False
        self.mutex = Lock()
        self.msg = HeartBeat()

    def start(self, frequency):
        self.frequency = frequency
        self.rate = rospy.Rate(frequency)
        self.heartbeat_publisher = rospy.Publisher(rospy.get_name() + '/heartbeat', HeartBeat, queue_size=10)
        self.msg.status = HeartbeatCode.STARTING
        self.msg.details = "STARTING"
        self.msg.header.stamp = rospy.Time.now()
        thread.start_new_thread(self.pub_thread, ())
        rospy.loginfo("[Heartbeat] publishing frequency: %d", self.frequency )
    
    def update_status(self, new_status, details):
        if self.heartbeat_publisher == None:
            rospy.logerr("Cannot publish on an uninitialized/stopped heartbeat topic")
            return
        self.msg.header.stamp = rospy.Time.now()
        if new_status != None:
            self.mutex.acquire()
            self.msg.status = new_status
            self.msg.details = details
            self.mutex.release()
        else:
            self.msg.status = HeartbeatCode.INVALID
            self.msg.details = "Invalid internal node state"
        
    def pub_thread(self):
        while not rospy.is_shutdown() and self.stop_heartbeat != True:
            self.mutex.acquire()
            if rospy.Time.now().to_sec() - rospy.Time.now().to_sec() >= 1/self.frequency:
                self.msg.status = HeartbeatCode.INTERNAL_ERROR
                self.msg.details = "Watchdog: the duration between two updates was too long, error"
                self.msg.header.stamp = rospy.Time.now()
            self.heartbeat_publisher.publish(self.msg)
            self.mutex.release()
            self.rate.sleep()


    def shutdown(self):
        self.stop_heartbeat = True

    
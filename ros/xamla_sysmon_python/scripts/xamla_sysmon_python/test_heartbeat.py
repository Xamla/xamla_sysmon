#!/usr/bin/env python
from heartbeat import *
import rospy
from std_msgs.msg import String

def talker():
    rospy.init_node('talker')
    chatter_pub = rospy.Publisher('chatter', String, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    heartbeat_client = HeartbeatClient()
    heartbeat_client.start(10)


    count = 0
    while not rospy.is_shutdown():

        msg = String()
        msg.data =  "hello world thread "
        rospy.loginfo("%s", msg.data)
        chatter_pub.publish(msg)

        if count == 0 :
            my_status = HeartbeatCode.STARTING
            my_details = "STARTING"
            rospy.loginfo('changed state to STARTING')
            heartbeat_client.update_status(my_status, my_details)
        elif count == 10 :
            my_status = HeartbeatCode.GO
            my_details = "GO"
            rospy.loginfo('changed state to GO')
            heartbeat_client.update_status(my_status, my_details)
        elif count == 20 :
            my_status = HeartbeatCode.INTERNAL_ERROR
            my_details = "INTERNAL_ERROR"
            rospy.loginfo('changed state to INTERNAL_ERROR')
            heartbeat_client.update_status(my_status, my_details)
        elif count == 30 :
            my_status = HeartbeatCode.EMERGENCY_STOP
            my_details = "EMERGENCY_STOP"
            rospy.loginfo('changed state to EMERGENCY_STOP')
            heartbeat_client.update_status(my_status, my_details)
        elif count == 40 :
            my_status = HeartbeatCode.SECONDARY_ERROR
            my_details = "SECONDARY_ERROR"
            rospy.loginfo('changed state to SECONDARY_ERROR')
            heartbeat_client.update_status(my_status, my_details)
        elif count == 50 :
            my_status = HeartbeatCode.INVALID
            my_details = "INVALID"
            rospy.loginfo('changed state to INVALID')
            heartbeat_client.update_status(my_status, my_details)
        elif count == 60 :
            heartbeat_client.shutdown()
        elif count == 100 :
            break

        count+=1

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

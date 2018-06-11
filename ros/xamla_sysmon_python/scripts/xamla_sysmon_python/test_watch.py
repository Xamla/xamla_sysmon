#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from watch import *

def talker():
    rospy.init_node('talker')
    chatter_pub = rospy.Publisher('chatter', String, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    watch_client = WatchClient(2, 5)
    watch_client.start()
    #rospy.spin()
    rospy.sleep(rospy.Duration(2))
    print "latest ---"
    glo_lat = watch_client.get_global_state()
    print glo_lat
    print "summary ---"
    glo_sum = watch_client.get_global_state_summary()
    print glo_sum.go
    print glo_sum.nogo
    print glo_sum.only_secondary_error
    print glo_sum.error_message
    print glo_sum.time_stamp
    
    rospy.sleep(rospy.Duration(3))
    watch_client.shutdown()
    #rospy.spin()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
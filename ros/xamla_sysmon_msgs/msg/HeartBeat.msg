# ROS standard header
std_msgs/Header header
# node status code (0-10 "OK" states, >10 error states)
#    0: idle / ready for operation
#    1: busy
#    2: global stop
#   10: unconfigured
#   11: emergency stop
#   12: hardware error
#   13: communication error
int32 status
# status details, might be empty
string details

# ROS standard header
std_msgs/Header header
# Overall status of the ROS system as bit mask
# all 0: ready to go
# 1: at least one service is not configured properly
# 2: at least one node reports errors
int32 system_status
# Array of the topics reporting not ready to go
string[] topics
# Corresponding list of errorcodes
# 1: unconfigured
# 2: hardware error
# 4: invalid config
int32[] err_code
# Description of the error, empty if everythink is OK
string[] topic_msg
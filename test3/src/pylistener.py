#!/usr/bin/env python
import rospy
import math
from test3.msg import gps

def callback(gps):
    distance = math.sqrt(math.pow(gps.x,2)+math.pow(gps.y,2))
    rospy.loginfo('distance = %f, state = %s, num is %d',distance,gps.state,gps.num)

def listener():
    rospy.init_node('pylistener')
    rospy.Subscriber('gps_info1',gps,callback,queue_size=1)
    rospy.spin()

if __name__=='__main__':
    listener()
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

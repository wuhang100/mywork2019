#!/usr/bin/env python
import rospy
import math
from test3.msg import gps

def callback(gps):
    distance = math.sqrt(math.pow(gps.x2,2)+math.pow(gps.y2,2))
    absx = rospy.get_param('globalx',default= 0.0)
    rospy.loginfo('distance = %f, state = %s, num is %d, absx is %f',distance,gps.state,gps.num,absx)

def listener():
    rospy.init_node('pylistener2')
    rospy.Subscriber('gps_info2',gps,callback,queue_size=1)
    rospy.spin()

if __name__=='__main__':
    listener()
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python
import rospy
from test3.msg import gps

def talker():
    rospy.init_node('pytalker',anonymous=True)
    pub = rospy.Publisher('gps_info1',gps,queue_size=2)
    pub2 = rospy.Publisher('gps_info2',gps,queue_size=2)
    rate = rospy.Rate(1)
    x = 1.0
    y = 1.0
    x2 = 2.0
    y2 = 2.0
    num = 1
    state = 'woking'
    while not rospy.is_shutdown():
        rospy.loginfo('talker %d',num)
        pub.publish(gps(state,x,y,x2,y2,num))
        pub2.publish(gps(state,x,y,x2,y2,num))
        num = num+1
        rate.sleep()

if __name__=='__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
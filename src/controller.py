#!/usr/bin/env python
# license removed for brevity
import rospy
from nav_msgs.msg import Odometry

def talker():
    pub = rospy.Publisher('/dataNavigator', Odometry, queue_size=1)
    rospy.init_node('controller', anonymous=True)
    rate = rospy.Rate(20) 
    begin = rospy.get_time()
    v = 0.5
    while not rospy.is_shutdown():
        msg = Odometry()
        msg.twist.twist.linear.x = v
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = -0.1
        now = rospy.get_time()
#        during = now - begin
#        if during > 20:
#            v = -v
#            begin = now
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
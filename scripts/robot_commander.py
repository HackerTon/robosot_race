#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time

def red_xy_callback():
    pass

def robot_commander():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.Subscriber("/red/colour_xy", String, red_xy_callback)
    rospy.init_node('commander', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    command = Twist()
    while not rospy.is_shutdown():
        #hello_str = "salam %s" % rospy.get_time()
        command.linear.x = 2.0  # m/s
        command.angular.z = 0.0 # rad/s
        rospy.loginfo(command)
        pub.publish(command)
        time.sleep(1.0)
        command.linear.x = 0.0  # m/s
        command.angular.z = 1.0 # rad/s
        rospy.loginfo(command)
        pub.publish(command)
        time.sleep(1.0)        
        rate.sleep()

if __name__ == '__main__':
    try:
        robot_commander()
    except rospy.ROSInterruptException:
        pass

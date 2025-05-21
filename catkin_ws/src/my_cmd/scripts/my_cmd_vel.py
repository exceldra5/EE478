#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, TwistStamped

latest_twist = Twist()

def cmd_vel_callback(msg):
    global latest_twist
    latest_twist = msg

def cmd_vel_teleop():
    rospy.init_node('my_cmd_vel', anonymous=True)

    pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)

    rate = rospy.Rate(20)
    
    rospy.loginfo("start my_cmd_vel node")
    
    while not rospy.is_shutdown():
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = rospy.Time.now()
        twist_stamped.twist = latest_twist
        pub.publish(twist_stamped)
        rate.sleep()

if __name__ == '__main__':
    try:
        cmd_vel_teleop()
    except rospy.ROSInterruptException:
        pass

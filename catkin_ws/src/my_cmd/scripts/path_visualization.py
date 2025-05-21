#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

path = Path()

def pose_callback(msg):
    global path
    pose = PoseStamped()
    pose.header = msg.header
    pose.pose = msg.pose
    path.header = msg.header
    path.poses.append(pose)

def visualize_path():
    rospy.init_node('path_visualizer', anonymous=True)
    path_pub = rospy.Publisher('/drone_path', Path, queue_size=10)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_callback)

    rate = rospy.Rate(20)
    rospy.loginfo("Path visualizer node started")

    while not rospy.is_shutdown():
        path_pub.publish(path)
        rate.sleep()

if __name__ == '__main__':
    try:
        visualize_path()
    except rospy.ROSInterruptException:
        pass

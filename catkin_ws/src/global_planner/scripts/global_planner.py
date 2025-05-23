#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
import math
import numpy as np

class GlobalPlanner:
    def __init__(self):
        rospy.init_node("global_planner")

        # Parameters
        self.lookahead_distance = rospy.get_param("~lookahead_distance", 1.0)
        
        # Tree position (assuming tree is at origin)
        self.tree_position = Point(0, 0, 0)
        
        # Generate circular waypoints around the tree
        radius = 3.0  # meters
        num_points = 8
        angles = np.linspace(0, 2*np.pi, num_points)
        
        self.global_waypoints = []
        for angle in angles:
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            z = 1.0  # Fixed height
            self.global_waypoints.append(Point(x, y, z))
            
        self.global_index = 0

        # Current state
        self.cur_position = None
        self.path = None

        # ROS Interfaces
        self.path_sub = rospy.Subscriber("/planning/bspline_path", Path, self.path_callback)
        # Changed to ORB SLAM pose topic
        self.pose_sub = rospy.Subscriber("/orb_slam3_ros/camera_pose", PoseStamped, self.pose_callback)

        self.local_pub = rospy.Publisher("/lookahead_waypoint", PoseStamped, queue_size=1)
        self.global_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

        self.arrival_threshold = 0.7

    def pose_callback(self, msg):
        self.cur_position = msg.pose.position
        self.publish_lookahead_point()
        self.check_and_publish_global()

    def path_callback(self, msg):
        self.path = msg.poses
        self.publish_lookahead_point()

    def publish_lookahead_point(self):
        if self.cur_position is None or self.path is None:
            return
        
        for pose_stamped in self.path:
            pt = pose_stamped.pose.position
            dist = self.euclidean_distance(pt, self.cur_position)
            if dist >= self.lookahead_distance:
                waypoint = PoseStamped()
                waypoint.header.stamp = rospy.Time.now()
                waypoint.header.frame_id = "odom"
                waypoint.pose.position = pt
                
                # Calculate orientation to face the tree
                dx = self.tree_position.x - pt.x
                dy = self.tree_position.y - pt.y
                yaw = math.atan2(dy, dx)
                
                # Convert yaw to quaternion
                waypoint.pose.orientation.w = math.cos(yaw/2)
                waypoint.pose.orientation.z = math.sin(yaw/2)
                
                self.local_pub.publish(waypoint)
                rospy.loginfo("[Local] Lookahead published: x=%.2f y=%.2f z=%.2f", pt.x, pt.y, pt.z)
                return
        
        rospy.logwarn("No lookahead point found")

    def check_and_publish_global(self):
        if self.cur_position is None or self.global_index >= len(self.global_waypoints):
            return

        goal = self.global_waypoints[self.global_index]
        dist = self.euclidean_distance(goal, self.cur_position)

        if dist < self.arrival_threshold:
            rospy.loginfo("Arrived at global waypoint %d", self.global_index)
            self.global_index += 1
            if self.global_index >= len(self.global_waypoints):
                rospy.loginfo("All global waypoints reached")
                return
            goal = self.global_waypoints[self.global_index]

        waypoint = PoseStamped()
        waypoint.header.stamp = rospy.Time.now()
        waypoint.header.frame_id = "odom"
        waypoint.pose.position = goal
        
        # Calculate orientation to face the tree
        dx = self.tree_position.x - goal.x
        dy = self.tree_position.y - goal.y
        yaw = math.atan2(dy, dx)
        
        # Convert yaw to quaternion
        waypoint.pose.orientation.w = math.cos(yaw/2)
        waypoint.pose.orientation.z = math.sin(yaw/2)
        
        self.global_pub.publish(waypoint)
        rospy.loginfo("[Global] Waypoint %d published: x=%.2f y=%.2f z=%.2f",
                      self.global_index, goal.x, goal.y, goal.z)

    @staticmethod
    def euclidean_distance(p1, p2):
        return math.sqrt(
            (p1.x - p2.x) ** 2 +
            (p1.y - p2.y) ** 2 +
            (p1.z - p2.z) ** 2
        )

if __name__ == '__main__':
    GlobalPlanner()
    rospy.spin()
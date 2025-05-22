#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
import math
import numpy as np
import tf

class GlobalPlanner:
    def __init__(self):
        rospy.init_node("global_planner")

        # Parameters
        self.lookahead_distance = rospy.get_param("~lookahead_distance", 1.0)
        self.scale_factor = rospy.get_param("~scale_factor", 1.2)  # Adjust this value based on your setup

        self.global_waypoints = [
            Point(5, -5, 0),
            Point(10, 0, 0),
            Point(5, 5, 0),
            Point(0, 0, 0),
        ]
        self.global_index = 0

        # Current state
        self.cur_position = None
        self.path = None
        self.rotation_step = 0  # 0, 1, or 2 for three steps
        self.target_yaw = 0.0
        self.current_yaw = 0.0

        # Change to ORB-SLAM3 pose topic
        # self.pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback)
        self.pose_sub = rospy.Subscriber("/orb_slam3/camera_pose", PoseStamped, self.pose_callback)

        self.local_pub = rospy.Publisher("/lookahead_waypoint", PoseStamped, queue_size=1)
        self.global_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

        self.arrival_threshold = 0.7

    def pose_callback(self, msg):
        # Apply scale factor to the current position
        self.cur_position = msg.pose.position
        self.cur_position.x *= self.scale_factor
        self.cur_position.y *= self.scale_factor
        self.cur_position.z *= self.scale_factor
        
        # Update current yaw from orientation
        quat = [msg.pose.orientation.x, msg.pose.orientation.y, 
                msg.pose.orientation.z, msg.pose.orientation.w]
        _, _, self.current_yaw = tf.transformations.euler_from_quaternion(quat)
        
        self.publish_lookahead_point()
        self.check_and_publish_global()

    def calculate_target_yaw(self, position):
        dx = 5 - position.x     # Tree is at (5,0,0)
        dy = 0 - position.y
        return math.atan2(dy, dx)

    def get_step_yaw(self, start_yaw, target_yaw, step):
        # Calculate the total rotation needed
        total_rotation = target_yaw - start_yaw
        # Normalize to [-pi, pi]
        total_rotation = math.atan2(math.sin(total_rotation), math.cos(total_rotation))
        # Calculate the step rotation (15 degrees = pi/12)
        step_rotation = total_rotation * (step + 1) / 3
        return start_yaw + step_rotation

    def publish_lookahead_point(self):
        if self.cur_position is None:
            return
        
        for pt in self.global_waypoints[self.global_index:]:
            dist = self.euclidean_distance(pt, self.cur_position)
            if dist >= self.lookahead_distance:
                waypoint = PoseStamped()
                waypoint.header.stamp = rospy.Time.now()
                waypoint.header.frame_id = "odom"
                waypoint.pose.position = pt

                # Calculate target yaw for the waypoint
                self.target_yaw = self.calculate_target_yaw(pt)
                
                # Calculate step yaw based on current rotation step
                step_yaw = self.get_step_yaw(self.current_yaw, self.target_yaw, self.rotation_step)
                
                # Update rotation step
                if dist < self.arrival_threshold * 2:  # Start rotation when close to waypoint
                    self.rotation_step = (self.rotation_step + 1) % 3
                else:
                    self.rotation_step = 0

                waypoint.pose.orientation.w = math.cos(step_yaw/2)
                waypoint.pose.orientation.z = math.sin(step_yaw/2)

                self.local_pub.publish(waypoint)
                rospy.loginfo("[Local] Lookahead published: x=%.2f y=%.2f z=%.2f yaw=%.2f° step=%d",
                            pt.x, pt.y, pt.z, math.degrees(step_yaw), self.rotation_step)
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
            self.rotation_step = 0  # Reset rotation step for new waypoint
            if self.global_index >= len(self.global_waypoints):
                rospy.loginfo("All global waypoints reached")
                return
            goal = self.global_waypoints[self.global_index]

        waypoint = PoseStamped()
        waypoint.header.stamp = rospy.Time.now()
        waypoint.header.frame_id = "odom"
        waypoint.pose.position = goal

        # Calculate target yaw for the goal
        self.target_yaw = self.calculate_target_yaw(goal)
        
        # Calculate step yaw based on current rotation step
        step_yaw = self.get_step_yaw(self.current_yaw, self.target_yaw, self.rotation_step)
        
        waypoint.pose.orientation.w = math.cos(step_yaw/2)
        waypoint.pose.orientation.z = math.sin(step_yaw/2)
        
        self.global_pub.publish(waypoint)
        rospy.loginfo("[Global] Waypoint %d published: x=%.2f y=%.2f z=%.2f yaw=%.2f° step=%d",
                      self.global_index, goal.x, goal.y, goal.z, math.degrees(step_yaw), self.rotation_step)

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
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

        self.global_waypoints = [
            Point(5, -5, 0),
            Point(10, 0, 0),
            Point(5, 5, 0),
            Point(0, 0, 0),
        ]
        self.global_index = 0

        # Current state
        self.cur_position = None
        self.prev_position = None  # Store previous position
        self.path = None
        self.tracking_fail_count = 0
        self.is_tracking_failed = False
        self.last_valid_position = None  # Store last valid position before tracking fail

        # Change to ORB-SLAM3 pose topic
        self.pose_sub = rospy.Subscriber("/orb_slam3/camera_pose", PoseStamped, self.pose_callback)

        self.local_pub = rospy.Publisher("/lookahead_waypoint", PoseStamped, queue_size=1)
        self.global_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

        self.arrival_threshold = 0.7
        self.tracking_fail_threshold = 3  # Number of consecutive fails before taking action
        self.position_change_threshold = 0.1  # Threshold to detect sudden position changes

    def pose_callback(self, msg):
        # Store previous position
        self.prev_position = self.cur_position
        self.cur_position = msg.pose.position

        # Check for tracking fail
        if self.detect_tracking_fail():
            self.handle_tracking_fail()
        else:
            self.tracking_fail_count = 0
            self.is_tracking_failed = False
            self.last_valid_position = self.cur_position
            self.publish_lookahead_point()
            self.check_and_publish_global()

    def detect_tracking_fail(self):
        if self.prev_position is None or self.cur_position is None:
            return True

        # Check if position is at origin (0,0,0)
        if (abs(self.cur_position.x) < 0.01 and 
            abs(self.cur_position.y) < 0.01 and 
            abs(self.cur_position.z) < 0.01):
            return True

        # Check for sudden position changes
        if self.prev_position is not None:
            dx = self.cur_position.x - self.prev_position.x
            dy = self.cur_position.y - self.prev_position.y
            dz = self.cur_position.z - self.prev_position.z
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            if dist > self.position_change_threshold:
                return True

        return False

    def handle_tracking_fail(self):
        self.tracking_fail_count += 1
        rospy.logwarn(f"Tracking fail detected! Count: {self.tracking_fail_count}")

        if self.tracking_fail_count >= self.tracking_fail_threshold:
            self.is_tracking_failed = True
            if self.last_valid_position is not None:
                # Publish return to last valid position
                waypoint = PoseStamped()
                waypoint.header.stamp = rospy.Time.now()
                waypoint.header.frame_id = "odom"
                waypoint.pose.position = self.last_valid_position
                waypoint.pose.orientation.w = 1.0
                self.local_pub.publish(waypoint)
                rospy.loginfo("Returning to last valid position")
            else:
                # If no valid position, hover in place
                if self.cur_position is not None:
                    waypoint = PoseStamped()
                    waypoint.header.stamp = rospy.Time.now()
                    waypoint.header.frame_id = "odom"
                    waypoint.pose.position = self.cur_position
                    waypoint.pose.orientation.w = 1.0
                    self.local_pub.publish(waypoint)
                    rospy.loginfo("Hovering in place due to tracking fail")

    def publish_lookahead_point(self):
        if self.cur_position is None or self.is_tracking_failed:
            return
        
        for pt in self.global_waypoints[self.global_index:]:
            dist = self.euclidean_distance(pt, self.cur_position)
            if dist >= self.lookahead_distance:
                waypoint = PoseStamped()
                waypoint.header.stamp = rospy.Time.now()
                waypoint.header.frame_id = "odom"
                waypoint.pose.position = pt
                # Calculate orientation to face the tree from current position
                dx = 5 - self.cur_position.x     # Tree is at (5,0,0)
                dy = 0 - self.cur_position.y
                yaw = math.atan2(dy, dx)
                waypoint.pose.orientation.w = math.cos(yaw/2)
                waypoint.pose.orientation.z = math.sin(yaw/2)

                self.local_pub.publish(waypoint)
                rospy.loginfo("[Local] Lookahead published: x=%.2f y=%.2f z=%.2f", pt.x, pt.y, pt.z)
                return
        
        rospy.logwarn("No lookahead point found")

    def check_and_publish_global(self):
        if self.cur_position is None or self.global_index >= len(self.global_waypoints) or self.is_tracking_failed:
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
        # Calculate orientation to face the tree from current position
        dx = 5 - self.cur_position.x
        dy = 0 - self.cur_position.y
        yaw = math.atan2(dy, dx)
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
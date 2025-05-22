#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
import math
import tf

class GlobalPlanner:
    def __init__(self):
        rospy.init_node("global_planner")

        # Parameters
        self.lookahead_distance = rospy.get_param("~lookahead_distance", 1.0)

        # Generate waypoints in a spiral pattern
        global_waypoints = []
        # Maximum and minimum of square size
        x_min, x_max = 1.0, 8.0
        y_min, y_max = -4.0, 4.0
        z_start = 2.0
        z_step = 1.5
        num_layers = 3  # total iteration

        for i in range(num_layers):
            z = z_start + i * z_step
            xy_diff = i * 0.8
            
            global_waypoints.append(Point(x_min + xy_diff, y_max - xy_diff, z))
            global_waypoints.append(Point(x_min + xy_diff, y_min + xy_diff, z))
            global_waypoints.append(Point(x_max - xy_diff, y_min + xy_diff, z))
            global_waypoints.append(Point(x_max - xy_diff, y_max - xy_diff, z))
        
        global_waypoints.append(Point(0.0, 0.0, 4.0))

        self.global_waypoints = global_waypoints

        # Tree center coordinates
        self.tree_center_x = 5.0
        self.tree_center_y = 0.0

        self.global_index = 0

        # Current state
        self.cur_position = None
        self.path = None

        # ROS Interfaces
        self.pose_sub = rospy.Subscriber("/orb_slam3/camera_pose", PoseStamped, self.pose_callback)
        self.local_pub = rospy.Publisher("/lookahead_waypoint", PoseStamped, queue_size=1)
        self.global_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

        self.arrival_threshold = 0.7

    def pose_callback(self, msg):
        self.cur_position = msg.pose.position
        self.publish_lookahead_point()
        self.check_and_publish_global()

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
                
                # Calculate yaw so that the drone faces the tree center
                dx = self.tree_center_x - pt.x
                dy = self.tree_center_y - pt.y
                yaw = math.atan2(dy, dx)
                quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
                waypoint.pose.orientation.x = quat[0]
                waypoint.pose.orientation.y = quat[1]
                waypoint.pose.orientation.z = quat[2]
                waypoint.pose.orientation.w = quat[3]
                
                self.local_pub.publish(waypoint)
                rospy.loginfo("[Local] Lookahead published: x=%.2f y=%.2f z=%.2f yaw=%.2f° dist=%.2f",
                                pt.x, pt.y, pt.z, math.degrees(yaw), dist)
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

        # Calculate yaw to face the tree center when the drone reaches the goal point
        dx = self.tree_center_x - goal.x
        dy = self.tree_center_y - goal.y
        yaw = math.atan2(dy, dx)
        quat = tf.transformations.quaternion_from_euler(0, 0, yaw)

        waypoint = PoseStamped()
        waypoint.header.stamp = rospy.Time.now()
        waypoint.header.frame_id = "odom"
        waypoint.pose.position = goal
        waypoint.pose.orientation.x = quat[0]
        waypoint.pose.orientation.y = quat[1]
        waypoint.pose.orientation.z = quat[2]
        waypoint.pose.orientation.w = quat[3]
        self.global_pub.publish(waypoint)
        rospy.loginfo(
            "[Global] Waypoint %d published: x=%.2f y=%.2f z=%.2f, yaw=%.2f°",
            self.global_index,
            goal.x,
            goal.y,
            goal.z,
            math.degrees(yaw),
        )

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
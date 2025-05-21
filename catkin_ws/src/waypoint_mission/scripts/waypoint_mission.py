#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Point, PoseStamped

# Waypoints [[X list], [Y list], [Z list]]
waypoint_list = [[0, 1, 1, 0, 0],
                [0, 0, 1, 1, 0],
                [1, 1, 1, 0, 1]]

class WaypointMission:
    def __init__(self):
        self.cur_waypoint_idx = -1
        self.cur_waypoint = Point()

        self.waypoint_server = rospy.Service("waypoint_mission_server", Empty, self.waypoint_service)
        self.waypoint_pub = rospy.Publisher("waypoint_mission", Point, queue_size = 10)
        self.cur_position_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.auto_arrive_checker_cb, queue_size=1)
    
    def waypoint_service(self, req):
        if(self.get_next_waypoint()):
            rospy.loginfo("Waypoint updated")
        else:
            rospy.loginfo("Waypoint update failed")
        return EmptyResponse()
    
    def get_next_waypoint(self):
        print(len(waypoint_list[0]))
        if(self.cur_waypoint_idx < len(waypoint_list[0]) - 1):
            self.cur_waypoint_idx = self.cur_waypoint_idx + 1

        else:
            print("All waypoint mission are already been finished")
            return False
        
        self.cur_waypoint.x = waypoint_list[0][self.cur_waypoint_idx]
        self.cur_waypoint.y = waypoint_list[1][self.cur_waypoint_idx]
        self.cur_waypoint.z = waypoint_list[2][self.cur_waypoint_idx]

        return True
    
    def auto_arrive_checker_cb(self, msg):
        threshold = 0.3
        current = msg.pose.position
        dist = ((self.cur_waypoint.x - current.x) ** 2 + 
                (self.cur_waypoint.y - current.y) ** 2 + 
                (self.cur_waypoint.z - current.z) ** 2) ** 0.5
        if(dist < threshold):
            if self.get_next_waypoint():
                print("successfully arrived, and move to next point")
            else:
                print("finish moving")
        
    def run(self):
        self.waypoint_pub.publish(self.cur_waypoint)

    def check_waypoint_list(self):
        for dim in range(0, len(waypoint_list) - 2):
            if (len(waypoint_list[dim]) != len(waypoint_list[dim+1])):
                return False
            
        return True
    
if __name__ == '__main__':
    rospy.init_node("waypoint_mission_node")
    rate = rospy.Rate(10)

    wp_mission = WaypointMission()
    if not wp_mission.check_waypoint_list():
        rospy.logerr("Waypoint is not well formed")
        quit()

    while not rospy.is_shutdown():
        wp_mission.run()
        rate.sleep()



#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
MAX_SPEED = 8.3 # in M/s corresponds to 30 kph

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)
        
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.current_pose = None
        self.waypoints = None
        
        rospy.spin()

    def pose_cb(self, msg):
        # Parse Position Update
        self.current_pose = msg.pose
        self.send_final_waypoints()

    def waypoints_cb(self, msg):
        # Initialize the waypoints
        if self.waypoints is None:
            self.waypoints = msg.waypoints

    def traffic_cb(self, msg):
        # For initial phase ignore traffic lights, later on flag traffic light
        pass

    def obstacle_cb(self, msg):
        # Parse Obstacles which are passed to us.
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist
        
    def get_circular_waypoints(self, startIT, endIT):
        if endIT > len(self.waypoints):
            ret_waypoints = self.waypoints[startIT:] + self.waypoints[:len(self.waypoints) - endIT]
        else:
            ret_waypoints = self.waypoints[startIT:endIT]
        return ret_waypoints

    def find_closest_waypoint(self):
        min_dist = None
        min_found = False
        for (i, waypoint) in enumerate(self.waypoints):
            waypoint_x = waypoint.pose.pose.position.x
            waypoint_y = waypoint.pose.pose.position.y
            # distance
            dist = math.sqrt((self.current_pose.position.x - waypoint_x) ** 2 + (self.current_pose.position.y - waypoint_y) ** 2)
            rospy.loginfo(dist)
            if min_dist is None:
                dist = min_dist
                min_loc = i
            elif dist < min_dist:
                min_dist = dist
                min_loc = i
        return min_loc


    def send_final_waypoints(self):
        if self.waypoints is None:
            return

        pos = self.find_closest_waypoint()
        rospy.loginfo("####")
        rospy.loginfo(pos)

        waypoints = self.get_circular_waypoints(pos, pos + LOOKAHEAD_WPS)
        
        for (i,waypoint) in enumerate(waypoints):
            self.set_waypoint_velocity(waypoints, i, MAX_SPEED)
        
        lane = Lane()
        lane.waypoints = waypoints
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        self.final_waypoints_pub.publish(lane)


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

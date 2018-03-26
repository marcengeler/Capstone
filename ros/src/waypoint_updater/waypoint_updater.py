#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
import time
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

LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number
MAX_SPEED = 30 #10 #8.3  # in M/s corresponds to 30 kph

PUBLISH_RATE = 20      # Publishing rate (Hz)

max_local_distance = 20.0      # Max waypoint distance we admit for a local minimum (m)
publish_on_light_change = True # Force publishing if next traffic light changes
debugging = True               # Set to False for release (not too verbose, but it saves some computation power)


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher(
            'final_waypoints', Lane, queue_size=1)

        # Add State variables
        self.base_waypoints = []  # List of waypoints, as received from /base_waypoints
        self.next_waypoint = None # Next waypoint in car direction
        self.current_pose = None
        self.waypoints = None
        self.prev_waypoint_min = None
        
        self.red_light_waypoint = None # Waypoint index of the next red light
        self.prev_red_light_waypoint = None
        
        # Parameters
        self.stop_on_red = rospy.get_param('~stop_on_red', True)      # Enable/disable stopping on red lights
        self.force_stop_on_last_waypoint = rospy.get_param('~force_stop_on_last_waypoint', True)   # Enable/disable stopping on last waypoint
        self.accel = rospy.get_param('~target_brake_accel', -2.)     # Target brake acceleration
        self.stop_distance = rospy.get_param('~stop_distance', 10.0)  # Distance (m) where car will stop before red light


        # Launch periodic publishing into /final_waypoints
        rate = rospy.Rate(PUBLISH_RATE)
        while not rospy.is_shutdown():
            self.send_final_waypoints()
            rate.sleep()

    def pose_cb(self, msg):
        # Parse Position Update
        self.current_pose = msg.pose


    def waypoints_cb(self, msg):
        # Initialize the waypoints
        if self.waypoints is None:
            self.waypoints = msg.waypoints

        self.base_waypoints = self.waypoints

    def traffic_cb(self, msg):
        # For initial phase ignore traffic lights, later on flag traffic light
        prev_red_light_waypoint = self.red_light_waypoint
        self.red_light_waypoint = msg.data if msg.data >= 0 else None


        if prev_red_light_waypoint != self.red_light_waypoint:
            if debugging:
                rospy.loginfo("TrafficLight changed: %s", str(self.red_light_waypoint))
            if publish_on_light_change:
                self.send_final_waypoints()


    def obstacle_cb(self, msg):
        # Parse Obstacles which are passed to us.
        pass

    def get_waypoint_velocity(self, waypoints, waypoint):
        return waypoints[waypoint].twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def decelerate(self, waypoints, stop_index, stop_distance):
        """
        Decelerate a list of wayponts so that they stop on stop_index
        """
        if stop_index <= 0:
            return
        dist = self.distance(waypoints, 0, stop_index)
        step = dist / stop_index
        # Generate waypoint velocity by traversing the waypoint list backwards:
        #  - Everything beyond stop_index will have velocity = 0
        #  - Before that, constant (de)cceleration is applied until reaching
        #    previous waypoint velocity.
        # We assume constant distance between consecutive waypoints for simplicity
        v = 0.
        d = 0.
        for idx in reversed(range(len(waypoints))):
            if idx < stop_index:
                d += step
                if d > self.stop_distance:
                    v = math.sqrt(2*abs(self.accel)*(d-stop_distance))
            if v < self.get_waypoint_velocity(waypoints, idx):
                self.set_waypoint_velocity(waypoints, idx, v)


    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt(
            (a.x - b.x)**2 + (a.y - b.y)**2 + (a.z - b.z)**2)
        for i in range(wp1, wp2 + 1):
            dist += dl(waypoints[wp1].pose.pose.position,
                       waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def get_circular_waypoints(self, startIT, endIT):
        if endIT > len(self.waypoints):
            ret_waypoints = self.waypoints[startIT:] + self.waypoints[:endIT - startIT]
            indices = range(startIT, len(self.waypoints)) + range(0, endIT - startIT + 1)
        else:
            ret_waypoints = self.waypoints[startIT:endIT]
            indices = range(startIT, endIT + 1)
        return ret_waypoints, indices

    def find_closest_waypoint(self):
        min_dist = None
        min_found = False
        for (i, waypoint) in enumerate(self.waypoints):
            waypoint_x = waypoint.pose.pose.position.x
            waypoint_y = waypoint.pose.pose.position.y
            dist = math.sqrt((self.current_pose.position.x - waypoint_x)
                             ** 2 + (self.current_pose.position.y - waypoint_y) ** 2)

            if min_dist is None:
                min_dist = dist
                min_loc = i
            elif dist < min_dist:
                min_dist = dist
                min_loc = i
        return min_loc
        
    def send_final_waypoints(self):
        if self.waypoints is None or self.current_pose is None:
            return

        pos = self.find_closest_waypoint()

        final_waypoints, waypoint_idx = self.get_circular_waypoints(pos, pos + LOOKAHEAD_WPS)

        if self.red_light_waypoint != None :
            if self.prev_red_light_waypoint != self.red_light_waypoint:
                self.prev_red_light_waypoint = self.red_light_waypoint
                rospy.logwarn("self.red_light_waypoint=" + str(self.red_light_waypoint))
            try:
                red_idx = waypoint_idx.index(self.red_light_waypoint)
            except:
                red_idx = 0
                rospy.logwarn("Red Light Not in Index" + str(self.red_light_waypoint) + " " + str(waypoint_idx))
            self.decelerate(final_waypoints, red_idx, self.stop_distance)
        else:
            red_idx = None
            for (i, waypoint) in enumerate(final_waypoints):
                self.set_waypoint_velocity(final_waypoints, i, MAX_SPEED)

        lane = Lane()
        lane.waypoints = final_waypoints

        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        self.final_waypoints_pub.publish(lane)

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

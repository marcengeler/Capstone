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

LOOKAHEAD_WPS = 50 #200  # Number of waypoints we will publish. You can change this number
MAX_SPEED = 8.3 # in M/s corresponds to 30 kph

PUBLISH_RATE = 20      # Publishing rate (Hz)

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher(
            'final_waypoints', Lane, queue_size=1)

        self.current_pose = None
        self.waypoints = None

        # Launch periodic publishing into /final_waypoints
        rate = rospy.Rate(PUBLISH_RATE)
        while not rospy.is_shutdown():
            #self.update_and_publish()
            self.send_final_waypoints()
            rate.sleep()
#
        #rospy.spin()

    def update_and_publish(self):
        """
        - Update next_waypoint based on current_pose and base_waypoints
        - Generate the list of the next LOOKAHEAD_WPS waypoints
        - Update velocity for them
        - Publish them to "/final_waypoints"
        """
        # 1. Find next_waypoint based on ego position & orientation
        if self._update_next_waypoint():

            # 2. Generate the list of next LOOKAHEAD_WPS waypoints
            num_base_wp = len(self.base_waypoints)
            last_base_wp = num_base_wp - 1
            waypoint_idx = [idx % num_base_wp for idx in range(
                self.next_waypoint, self.next_waypoint + LOOKAHEAD_WPS)]
            final_waypoints = [self.base_waypoints[wp] for wp in waypoint_idx]

            # 3. If there is a red light ahead, update velocity for them
            if self.stop_on_red:
                # Start from original velocities
                self.restore_velocities(waypoint_idx)
                try:
                    red_idx = waypoint_idx.index(self.red_light_waypoint)
                    self.decelerate(final_waypoints, red_idx,
                                    self.stop_distance)
                except ValueError:
                    # No red light available: self.red_light_waypoint is None
                    # or not in final_waypoints
                    red_idx = None
                if debugging:
                    v = self.get_waypoint_velocity(final_waypoints, 0)
                    rospy.loginfo(
                        "Target velocity: %.1f, RL:%s wps ahead", v, str(red_idx))

            # 3b. If we are close to the end of the circuit, make sure that we
            # stop there
            if self.force_stop_on_last_waypoint or self.base_wp_orig_v[-1] < 1e-5:
                try:
                    last_wp_idx = waypoint_idx.index(last_base_wp)
                    self.decelerate(final_waypoints, last_wp_idx, 0)
                except ValueError:
                    # Last waypoint is not one of the next LOOKAHEAD_WPS
                    pass

            # 4. Publish waypoints to "/final_waypoints"
            self.publish_msg(final_waypoints)

    def publish_msg(self, final_waypoints):
        waypoint_msg = Lane()
        waypoint_msg.header.seq = self.msg_seq
        waypoint_msg.header.stamp = rospy.Time.now()
        waypoint_msg.header.frame_id = '/world'
        waypoint_msg.waypoints = final_waypoints
        self.final_waypoints_pub.publish(waypoint_msg)
        self.msg_seq += 1

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
        dl = lambda a, b: math.sqrt(
            (a.x - b.x)**2 + (a.y - b.y)**2 + (a.z - b.z)**2)
        for i in range(wp1, wp2 + 1):
            dist += dl(waypoints[wp1].pose.pose.position,
                       waypoints[i].pose.pose.position)
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
            dist = math.sqrt((self.current_pose.position.x - waypoint_x)
                             ** 2 + (self.current_pose.position.y - waypoint_y) ** 2)

            if min_dist is None:
                min_dist = dist
                min_loc = i
            elif dist < min_dist:
                min_dist = dist
                min_loc = i
                #min_found = True
            #elif min_found:
            #    break
        return min_loc


    def send_final_waypoints(self):
        #if self.waypoints is None:
        if self.waypoints is None or self.current_pose is None:
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

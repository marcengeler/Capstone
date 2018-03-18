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

LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number
MAX_SPEED = 30 #10 #8.3  # in M/s corresponds to 30 kph

PUBLISH_RATE = 20      # Publishing rate (Hz)


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

        self.red_light_waypoint = None # Waypoint index of the next red light
        #self.msg_seq = 0 # Sequence number of /final_waypoints message
        
        # Parameters
        self.stop_on_red = rospy.get_param('~stop_on_red', True)      # Enable/disable stopping on red lights
        self.force_stop_on_last_waypoint = rospy.get_param('~force_stop_on_last_waypoint', True)   # Enable/disable stopping on last waypoint
        self.accel = rospy.get_param('~target_brake_accel', -1.)     # Target brake acceleration
        self.stop_distance = rospy.get_param('~stop_distance', 5.0)  # Distance (m) where car will stop before red light


        # Launch periodic publishing into /final_waypoints
        rate = rospy.Rate(PUBLISH_RATE)
        while not rospy.is_shutdown():
            #self.update_and_publish()
            self.send_final_waypoints()
            rate.sleep()
#
        #rospy.spin()




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
        rospy.logwarn("self.red_light_waypoint: %s", str(self.red_light_waypoint))


        if prev_red_light_waypoint != self.red_light_waypoint:
            if debugging:
                rospy.loginfo("TrafficLight changed: %s", str(self.red_light_waypoint))
            if publish_on_light_change:
                #self.update_and_publish() # Refresh if next traffic light has changed
                self.send_final_waypoints()


    def obstacle_cb(self, msg):
        # Parse Obstacles which are passed to us.
        pass

        #get_waypoint_velocity(self, waypoints, waypoint):
    def get_waypoint_velocity(self, waypoints, waypoint):
        return waypoints[waypoint].twist.twist.linear.x
        #return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity
        rospy.logwarn("velocity="+str(velocity))

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


            #rospy.logwarn("startIT ="+str(startIT)+" ,endIT="+str(endIT)+" ,len(self.waypoints)="+str(len(self.waypoints))+" ,len(self.waypoints) - endIT="+str(len(self.waypoints) - endIT)+" ,endIT - startIT="+str(endIT - startIT))


            ret_waypoints = self.waypoints[startIT:] + self.waypoints[:endIT - startIT]
        else:
            #rospy.logwarn("startIT ="+str(startIT)+" ,endIT="+str(endIT)+" ,len(self.waypoints)="+str(len(self.waypoints))+" ,len(self.waypoints) - endIT="+str(len(self.waypoints) - endIT)+" ,endIT - startIT="+str(endIT - startIT))
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
        return min_loc

    def _update_next_waypoint(self):
        """
        Update next_waypoint based on base_waypoints and current_pose.
        @return True if a valid waypoint has been updated, False otherwise
        """
        if not self.base_waypoints:
            #rospy.logwarn("Waypoints not updated: base_waypoints not available yet.")
            return False


        if not self.current_pose:
            #rospy.logwarn("Waypoints not updated: current_pose not available yet.")
            return False

        # Get ego car variables
        ego_x = self.current_pose.position.x
        ego_y = self.current_pose.position.y
        ego_theta = math.atan2(self.current_pose.orientation.y, self.current_pose.orientation.x)

        # If we do have a next_waypoint, we start looking from it, and we stop looking
        # as soon as we get a local minimum. Otherwise we do a full search across the whole track
        t = time.time()
        wp = None
        yaw = 0
        dist = 1000000 # Long number
        if self.next_waypoint:
            idx_offset = self.next_waypoint
            full_search = False
        else:
            idx_offset = 0
            full_search = True
        num_base_wp = len(self.base_waypoints)

        for i in range(num_base_wp):
            idx = (i + idx_offset)%(num_base_wp)
            wp_x = self.base_waypoints[idx].pose.pose.position.x
            wp_y = self.base_waypoints[idx].pose.pose.position.y
            wp_d = math.sqrt((ego_x - wp_x)**2 + (ego_y - wp_y)**2)

            if wp_d < dist:
                dist = wp_d
                wp = idx
                if debugging:
                    # Angle betwee car heading and waypoint heading
                    yaw = math.atan2(wp_y - ego_y, wp_x - ego_x) - ego_theta
            elif not full_search:
                # Local minimum. If the waypoint makes sense, just use it and break
                if dist < max_local_distance:
                    break; # We found a point
                else:
                    # We seem to have lost track. We search again
                    rospy.logwarn("Waypoint updater lost track (local min at %.1f m after %d waypoints). Going back to full search.", dist, i+1)
                    full_search = True

        if debugging:
            rospy.loginfo("New next wp [%d] -> (%.1f,%.1f) after searching %d points in %fs", wp, dist * math.cos(yaw), dist * math.sin(yaw), i, time.time()-t)

        if wp is None:
            rospy.logwarn("Waypoint updater did not find a valid waypoint")
            return False

        self.next_waypoint = wp
        #rospy.logwarn("Waypoint_updater.py: _update_next_waypoint(): self.next_waypoint="+str(self.next_waypoint))

        return True

        
    def send_final_waypoints(self):
        #if self.waypoints is None:
        if self.waypoints is None or self.current_pose is None:
            return

        pos = self.find_closest_waypoint()

        #rospy.loginfo("####")
        #rospy.loginfo(pos)
        #272
        #rospy.logwarn("pos=" +str(pos))

        waypoints = self.get_circular_waypoints(pos, pos + LOOKAHEAD_WPS)
        #final_waypoints = self.get_circular_waypoints(pos, pos + LOOKAHEAD_WPS)
        
        # 50
        #rospy.logwarn("length of waypoints =" +str(len(waypoints)))

        #for waypoint in waypoints:
        #    rospy.logwarn("waypoint.twist.twist.linear.x =" +str(waypoint.twist.twist.linear.x))

        #10902
        num_base_wp = len(self.base_waypoints) 
        #rospy.logwarn("waypoint_updater.py: send_final_waypoints(): num_base_wp=" + str(num_base_wp))
        waypoint_idx = [idx % num_base_wp for idx in range(pos,pos+LOOKAHEAD_WPS)]
        #for i in enumerate(waypoint_idx):
        rospy.logwarn("waypoint_idx["+str(min(waypoint_idx))+" : "+str(max(waypoint_idx))+"]")


        
        final_waypoints = [self.base_waypoints[wp] for wp in waypoint_idx]
        if self.red_light_waypoint != None:
            rospy.logwarn("self.red_light_waypoint=" + str(self.red_light_waypoint))
            red_idx = waypoint_idx.index(self.red_light_waypoint)
            #rospy.logwarn("red_idx=:"+str(red_idx))

            self.decelerate(final_waypoints, red_idx, self.stop_distance)
        else:
            red_idx = None
            for (i, waypoint) in enumerate(final_waypoints):
                self.set_waypoint_velocity(final_waypoints, i, MAX_SPEED)


        # fixed twist velocity = MAX_SPEED = 10 ( m/s ?)
        #for (i, waypoint) in enumerate(waypoints):
            #rospy.logwarn("i="+str(i)+" ,waypoint.twist.twist.linear.x="+str(waypoint.twist.twist.linear.x))
            #self.set_waypoint_velocity(waypoints, i, MAX_SPEED)
            #v = self.get_waypoint_velocity(waypoints, i)
            #rospy.logwarn("i="+str(i)+" ,v="+str(v))


        lane = Lane()
        #waypoint_msg = Lane()


        #if debugging:
        #    v = self.get_waypoint_velocity(lane, 0)
        #    #rospy.loginfo("Target velocity: %.1f, RL:%s wps ahead", v, str(red_idx))
        #    rospy.loginfo("Target velocity: %.1f  ", v)


        #lane.header.seq = self.msg_seq
        #waypoint_msg.header.seq = self.msg_seq

        #lane.waypoints = waypoints # <== self.get_circular_waypoints(pos, pos + LOOKAHEAD_WPS)
        lane.waypoints = final_waypoints


        #lane.waypoints = final_waypoints
        #waypoint_msg.waypoints = final_waypoints

        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        self.final_waypoints_pub.publish(lane)

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

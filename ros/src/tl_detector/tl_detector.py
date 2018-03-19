#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
import sys
import numpy as np
import time


#STATE_COUNT_THRESHOLD = 3
STATE_COUNT_THRESHOLD = 1
#Traffic Light detection range
MEASURE_PERFORMANCE = True
TL_DETECTION_RANGE = 50
ProcessingTimeSum = 0
ProcessingIterations = 0

REDUCE_FREQ = True

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.img_rx_time = time.time()
        rospy.loginfo("TLDetector init at:"+str(self.img_rx_time))

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, msg):
        self.waypoints = msg.waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        global ProcessingTimeSum, ProcessingIterations
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        if REDUCE_FREQ: # only start to detect image every 500ms
            rx_time = time.time()
            #rospy.loginfo("new img rx at:"+str(rx_time))
            if rx_time - self.img_rx_time < 2: #ms ??
                return
            else:
                self.img_rx_time = time.time()
                rospy.loginfo("more than 200ms ??, start detect----------")

        if MEASURE_PERFORMANCE:
            startTime = time.time()

        self.has_image = True
        self.camera_image = msg
        # start to call classification:
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

        if MEASURE_PERFORMANCE:
            endTime = time.time()
            duration = endTime-startTime
            ProcessingTimeSum += duration
            ProcessingIterations += 1
            rospy.loginfo("Processing time of image_cb(): " + str(duration) + " average: " + str(ProcessingTimeSum/ProcessingIterations))

###########################################################################################################################
    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """

        # If we don't have any waypoints return None
        if self.waypoints is None:
            return

        # Get current position
        x = pose.position.x
        y = pose.position.y

        # define minimum distance variable
        minimum_distance = None
        minumum_location = None

        # Search through all the waypoints to get the closes waypoint
        for i,waypoint in enumerate(self.waypoints):
            waypoint_x = waypoint.pose.pose.position.x
            waypoint_y = waypoint.pose.pose.position.y

            dist_to_waypoint = math.sqrt((waypoint_x - x) ** 2 + (waypoint_y - y) ** 2)

            # Initialize minimum distance at first value, or get new minimum distance
            if minimum_distance is None:
                minimum_location = i
                minimum_distance = dist_to_waypoint
            elif dist_to_waypoint <= minimum_distance:
                minimum_location = i
                minimum_distance = dist_to_waypoint

        return minimum_location

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def get_distance(self, wp1_idx, wp2_idx):
        """Determines the distance between two points

        Args:
            wp1_idx (waypoint index): waypoint index 1
            wp2_idx (waypoint index): waypoint index 2

        Returns:
            int: distance in meters between two waypoint index

        """
        if(wp1_idx < 0 or wp1_idx > len(self.waypoints) or
           wp2_idx < 0 or wp2_idx > len(self.waypoints)):
           return -1

        wp1 = self.waypoints[wp1_idx].pose.pose.position
        wp2 = self.waypoints[wp2_idx].pose.pose.position
        dist = math.sqrt((wp1.x - wp2.x) ** 2 + (wp1.y - wp2.y) ** 2)
        return dist

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closest to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        closest_light = -1
        car_position = -1
        distance_to_tl = -1
        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_light_positions = self.config['stop_line_positions']

        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)

        # Loop through all the stop light positions
        for i,stop_light_position in enumerate(stop_light_positions):
            # Initialize a Pose
            light_pose = Pose()
            light_pose.position.x = stop_light_position[0]
            light_pose.position.y = stop_light_position[1]

            # Initialize a Waypoint, which is closest to the traffic light
            light_waypoint = self.get_closest_waypoint(light_pose)

            # Find the closest waypoint with a traffic light (or closest traffic light)
            if light_waypoint >= car_position:
                if light is None:
                    closest_light = light_waypoint
                    light = light_pose
                elif light_waypoint < closest_light:
                    closest_light = light_waypoint
                    light = light_pose

        if car_position and light:
            waypoint_num_to_light = abs(car_position - closest_light)
            distance_to_tl = self.get_distance(car_position, closest_light)

        if light:
            state = TrafficLight.UNKNOWN
            if (distance_to_tl < TL_DETECTION_RANGE):
                state = self.get_light_state(light)

                #rospy.loginfo("---------"+str(state)+"-------------")

                #rospy.loginfo("-dist to closest TL- " + str(distance_to_tl))
            return closest_light, state
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')

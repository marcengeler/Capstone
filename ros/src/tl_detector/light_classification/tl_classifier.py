from styx_msgs.msg import TrafficLight

import tensorflor as tf
import numpy as np
import cv2
import keras
from PIL import Image

class TLClassifier(object):
    def __init__(self):
        self.labels = [0, 1, 2]
		self.size = 25
		

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        return TrafficLight.UNKNOWN

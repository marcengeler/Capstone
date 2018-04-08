from yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID

import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        self.time = None
        self.max_vel = kwargs['max_vel']
        self.accel_limit = kwargs['accel_limit']
        self.decel_limit = kwargs['decel_limit']
        self.TP1_throttle = LowPassFilter(0.5, 0.1)

        self.throttle_control = PID(
            kp= 0.8,
            ki= 0.25,
            kd= 0.05,
            mn= -1.0,
            mx= 1.0)

        self.steer_control = PID(
            kp=1.0,
            ki=0.2,
            kd=0.0)

        self.steering_control = YawController(kwargs['wheel_base'], kwargs['steer_ratio'],
                                              kwargs['min_speed'], kwargs['max_lat_accel'],
                                              kwargs['max_steer_angle']
                                              )

    def control(self, *args, **kwargs):
        linear_velocity = kwargs['target_linear_velocity']
        angular_velocity = kwargs['target_angular_velocity']
        current_velocity = kwargs['current_linear_velocity']

        dbw_state = kwargs['dbw_state']

        if self.time is None or not dbw_state:
            self.time = rospy.get_time()
            return 0.0, 0.0, 0.0
            
        dt = rospy.get_time() - self.time

        steer = self.steering_control.get_steering(linear_velocity, angular_velocity, current_velocity)
        steer = self.steer_control.step(steer, dt)

        linear_velocity_error = linear_velocity - current_velocity
        controller_error = linear_velocity_error if (linear_velocity_error > 0) else (-linear_velocity_error)

        throttle = self.throttle_control.step(controller_error, dt)

        # We need throttle can be minus, i.e. < 0, when car needs slow down.
        if linear_velocity_error < 0:
            throttle = 0.0
            brake = abs(throttle)
        else:
            brake = 0.0

        rospy.logwarn("linear_velocity_error: " + str(linear_velocity_error))
        rospy.logwarn("controller_error: " + str(controller_error))
        rospy.logwarn("throttle: " + str(throttle))
        rospy.logwarn("brake: " + str(brake))
        
        self.time = rospy.get_time()
        return throttle, brake, steer

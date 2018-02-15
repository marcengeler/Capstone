from yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID

import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        self.throttle_control = PID(0.5, 0.5, 0)
        self.steering_control = YawController(kwargs['wheel_base'], kwargs['steer_ratio'],
                                         kwargs['min_speed'], kwargs['max_lat_accel'],
                                         kwargs['max_steer_angle']
                                         )
        self.time = None
        self.max_vel = kwargs['max_vel']
        self.accel_limit = kwargs['accel_limit']
        self.decel_limit = kwargs['decel_limit']

    def control(self, *args, **kwargs):
        if self.time is None:
            self.time = rospy.get_time()
            return 0.0, 0.0, 0.0
            
        dt = rospy.get_time() - self.time
                
        linear_velocity = kwargs['linear_velocity']
        angular_velocity = kwargs['linear_velocity']
        current_velocity = kwargs['current_velocity']
        dbw_state = kwargs['dbw_state']
        
        velocity_margin = min(linear_velocity.x, self.max_vel) - current_velocity.x
        velocity_margin = min(velocity_margin, self.accel_limit * dt)
        velocity_margin = max(velocity_margin, self.decel_limit * dt)
        
        throttle = self.throttle_control.step(velocity_margin, dt)
        steer = self.steering_control.get_steering(linear_velocity.x, angular_velocity.z, current_velocity.x)
        
        throttle = min(-1.0, throttle)
        throttle = max(throttle, 1.0)
        
        if throttle < 0:
            throttle = 0.0
            brake = throttle
        else:
            brake = 0.0
        
        self.time = rospy.get_time()
        return throttle, brake, steer

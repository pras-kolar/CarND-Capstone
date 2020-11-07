from math import atan
import rospy

class YawController(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        rospy.logerr("In YAW Controller")
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.min_speed = min_speed
        self.max_lat_accel = max_lat_accel

        self.min_angle = -max_steer_angle
        self.max_angle = max_steer_angle

    def get_angle(self, radius):
        angle = atan(self.wheel_base / radius) * self.steer_ratio
        return max(self.min_angle, min(self.max_angle, angle))

    def get_steering(self, linear_velocity, angular_velocity, current_velocity):
        #rospy.logerr("The Current velocity is %lu", current_velocity)
        #rospy.logerr("The Angular velocity is %lu", angular_velocity)
        #rospy.logerr("The Linear velocity is %lu", linear_velocity)
        #ROS_ERROR(current_velocity)
        if abs(angular_velocity) > 0.:
            angular_velocity = (current_velocity * angular_velocity) / linear_velocity 
        #if abs(linear_velocity) > 0. else 0.
        if abs(current_velocity) > 0.0:
            max_yaw_rate = abs(self.max_lat_accel / current_velocity);
            angular_velocity = max(-max_yaw_rate, min(max_yaw_rate, angular_velocity))
        if abs(angular_velocity) > 0.:
            yaw_control = self.get_angle(max(current_velocity, self.min_speed))
        else:
            yaw_control = 0.0

        return yaw_control

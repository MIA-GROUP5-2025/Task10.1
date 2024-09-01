#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float64

class YawKalmanFilter:
    def __init__(self):
        self.angle_estimate = 0.0  # Initial estimate of the YAW angle
        self.error_estimate = 1.0  # Initial estimate of the error in the angle
        self.process_noise = 0.01  # Process noise covariance (Q)
        self.measurement_noise = 0.1  # Measurement noise covariance (R)

        self.yaw_pub = rospy.Publisher('/filtered_yaw', Float64, queue_size=10)
        rospy.Subscriber('/imu_yaw_deg', Float32, self.yaw_callback)

    def yaw_callback(self, data):
        measured_yaw = data.data

        # Apply Kalman Filter
        self.angle_estimate = self.angle_estimate  # Predicted state
        self.error_estimate += self.process_noise  # Update error estimate

        kalman_gain = self.error_estimate / (self.error_estimate + self.measurement_noise)
        self.angle_estimate = self.angle_estimate + kalman_gain * (measured_yaw - self.angle_estimate)
        self.error_estimate = (1 - kalman_gain) * self.error_estimate

        # Publish the filtered YAW angle
        self.yaw_pub.publish(self.angle_estimate)

if __name__ == '__main__':
    rospy.init_node('yaw_kalman_filter')
    YawKalmanFilter()
    rospy.spin()


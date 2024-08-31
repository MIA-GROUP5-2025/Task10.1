#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

def quaternion_to_euler_angle(w, x, y, z):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z

def imu_callback(data):
    roll, pitch, yaw = quaternion_to_euler_angle(data.orientation.w, data.orientation.x, data.orientation.y, data.orientation.z)
    yaw_deg = yaw * 180.0 / math.pi
    pub.publish(yaw_deg)

if __name__ == '__main__':
    rospy.init_node('imu_converter', anonymous=True)
    rospy.Subscriber('/imu', Imu, imu_callback)
    pub = rospy.Publisher('/imu_yaw_deg', Float32, queue_size=10)
    rospy.spin()

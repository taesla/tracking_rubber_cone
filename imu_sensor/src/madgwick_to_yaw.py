#!/usr/bin/env python

import rospy
import numpy as np
import math
import time

from std_msgs.msg import Header, Float64, Int8
from sensor_msgs.msg import Imu

madg_pub=rospy.Publisher('/madgwick_yaw',Float64, queue_size=1)

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
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
     
        return yaw_z # in radians

def callback(msg):

	qx=msg.orientation.x
	qy=msg.orientation.y
	qz=msg.orientation.z
	qw=msg.orientation.w

	yaw = euler_from_quaternion(qx,qy,qz,qw)
	deg_yaw = yaw * 180 / np.pi

	madg_pub.publish(deg_yaw)


if __name__=='__main__':
	
	rospy.init_node('madgwick_yaw')

	gyr_sub=rospy.Subscriber("/imu/data",Imu,callback)

	rospy.spin()

	

	

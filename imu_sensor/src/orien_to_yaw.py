#!/usr/bin/env python

import rospy
import numpy as np
import math
import time

from std_msgs.msg import Header, Float64, Int8
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

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

def xsens_callback(msg):
	global xsens_yaw
	qx=msg.orientation.x
	qy=msg.orientation.y
	qz=msg.orientation.z
	qw=msg.orientation.w

	yaw = euler_from_quaternion(qx,qy,qz,qw)
	deg_yaw = yaw * 180 / np.pi
	xsens_yaw = deg_yaw
	#xsens_pub.publish(xsens_msg)

def e2box_callback(msg):
	global e2box_yaw
	qx=msg.orientation.x
	qy=msg.orientation.y
	qz=msg.orientation.z
	qw=msg.orientation.w

	yaw = euler_from_quaternion(qx,qy,qz,qw)
	deg_yaw = yaw * 180 / np.pi
	e2box_yaw = deg_yaw
	#e2box_pub.publish(e2box_msg)

def gps_callback(msg):
	global gps_yaw
	qx=msg.pose.pose.orientation.x
	qy=msg.pose.pose.orientation.y
	qz=msg.pose.pose.orientation.z
	qw=msg.pose.pose.orientation.w

	yaw = euler_from_quaternion(qx,qy,qz,qw)
	deg_yaw = yaw * 180 / np.pi
	gps_yaw = deg_yaw
	#gps_pub.publish(gps_msg)


if __name__=='__main__':
	gps_yaw = 0
	e2box_yaw = 0
	xsens_yaw = 0

	rospy.init_node('yaw_transform')

	rospy.Subscriber("/imu/data",Imu,xsens_callback,queue_size=1)
	rospy.Subscriber("/imu/data_raw",Imu,e2box_callback,queue_size=1)
	rospy.Subscriber("/odom_gps",Odometry, gps_callback,queue_size=1)
	
	xsens_pub = rospy.Publisher("/xsens_yaw",Float64,queue_size=1)
	e2box_pub = rospy.Publisher("/e2box_yaw",Float64,queue_size=1)
	gps_pub = rospy.Publisher("/gps_yaw",Float64,queue_size=1)

	xsens_msg = Float64()
	e2box_msg = Float64()
	gps_msg = Float64()

	#rospy.spin()

	r = rospy.Rate(10)

	while not rospy.is_shutdown():
		gps_pub.publish(gps_yaw)
		e2box_pub.publish(e2box_yaw)
		xsens_pub.publish(xsens_yaw)
		r.sleep()

	

	

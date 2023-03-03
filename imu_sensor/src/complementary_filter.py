#!/usr/bin/env python

import rospy
import numpy as np
import math
import time
import message_filters

from std_msgs.msg import Header, Float64, Int8
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float32, Float64

filtered_yaw = 0
old_time = 0
i = 0

yaw_pub= rospy.Publisher('/madgwick_yaw',Float64,queue_size=5)

def callback(gyr,mag):

	global filtered_yaw, old_time, i

	t=gyr.header.stamp
	time=t.to_sec()
	delta_time=time-old_time
	old_time=time

	alpha=0
	
	mag_angle = math.atan2( -mag.magnetic_field.x , -mag.magnetic_field.y )
	gyr_angle = filtered_yaw + gyr.angular_velocity.z * delta_time

	if i==0:
		filtered_yaw=mag_angle

	else:
		gyr_angle = filtered_yaw + gyr.angular_velocity.z * delta_time

		filtered_yaw = alpha * gyr_angle + ( 1 - alpha ) * mag_angle

	i=i+1
	deg_yaw = filtered_yaw * 180 / np.pi
	yaw_pub.publish(deg_yaw)

if __name__=='__main__':
	
	rospy.init_node('complementary_filter')

	gyr_sub=message_filters.Subscriber("/imu_data",Imu)
	mag_sub=message_filters.Subscriber("/imu_mag",MagneticField)
	
	ts= message_filters.ApproximateTimeSynchronizer([gyr_sub,mag_sub],1,0.1)

	ts.registerCallback(callback)

	rospy.spin()

	

	

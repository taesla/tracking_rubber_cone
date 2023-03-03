#! /usr/bin/env python

import rospy
import numpy as np
import math
from nav_msgs.msg import Path
from std_msgs.msg import Float32
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

from ackermann_msgs.msg import AckermannDriveStamped
# paramters
dt = 0.1

# k =0.5
k = 1  # control gain

# ERP42 PARAMETERS
LENGTH = 1.600
WIDTH = 1.160

# # GV70 PARAMETERS
# LENGTH = 4.715
# WIDTH = 1.910
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

def normalize_angle(angle):
	while angle > np.pi:
		angle -= 2.0 * np.pi

	while angle < -np.pi:
		angle += 2.0 * np.pi

	return angle


def stanley_control(x, y, yaw, v, map_xs, map_ys, map_yaws, L):
	# find nearest point
	global k
	min_dist = 1e9
	min_index = 0
	n_points = len(map_xs)
	front_x = x + L * np.cos(yaw)
	front_y = y + L * np.sin(yaw)

	for i in range(n_points):
		dx = front_x - map_xs[i]
		dy = front_y - map_ys[i]

		dist = np.sqrt(dx * dx + dy * dy)
		if dist < min_dist:
			min_dist = dist
			min_index = i
	# compute cte at front axle
	map_x = map_xs[min_index]
	map_y = map_ys[min_index]
	map_yaw = map_yaws[min_index]
	dx = map_x - front_x
	dy = map_y - front_y

	perp_vec = [np.cos(yaw + np.pi/2), np.sin(yaw + np.pi/2)]
	cte = np.dot([dx, dy], perp_vec) # Cross track error

	# control law
#	yaw_term = normalize_angle(map_yaw - yaw) * np.sin(np.pi/2 / (1+v/5))
	yaw_term = normalize_angle(map_yaw - yaw) #heading error
	cte_term = np.arctan2(k*cte, v) # cross track error
	w_yaw = 1
	w_cte = 1
	# steering
	steer = w_yaw * yaw_term + w_cte * cte_term
	
	return steer#, [w_yaw, w_cte, k, yaw_term, cte_term]



def callback(msg):
	map_x = []
	map_y = []
	map_yaw = []

	for i in range(len(msg.poses)):
		map_x.append(msg.poses[i].pose.position.x)
		map_y.append(msg.poses[i].pose.position.y)
		map_yaw.append(euler_from_quaternion(msg.poses[i].pose.orientation.x,msg.poses[i].pose.orientation.y,msg.poses[i].pose.orientation.z,msg.poses[i].pose.orientation.w))


	steer = stanley_control(0, 0, 0, 1.05, map_x, map_y, map_yaw, 0)

	pub.publish(steer)

if __name__ == '__main__':
	rospy.init_node('stanley_control')
	rospy.Subscriber('/waypoints',Path,callback)
	pub = rospy.Publisher('/steer',Float32,queue_size=1)
	rospy.spin()

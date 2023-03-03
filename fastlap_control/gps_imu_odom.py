#!/usr/bin/env python

import rospy
import serial
import time
import math
import message_filters
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import QuaternionStamped
from nav_msgs.msg import Odometry
from pyproj import Proj, transform

import numpy as np

def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]

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
     
        return roll_x, pitch_y, yaw_z # in radians


def callback(gps, imu):
	
    cov1 = gps.position_covariance[0] 
    cov2 = gps.position_covariance[4]
    cov3 = gps.position_covariance[8]  
    kcity=Proj(init='epsg:5179')
    wgs84=Proj(init='epsg:4326')
    a,b=transform(wgs84,kcity,gps.longitude,gps.latitude)
    gpose.pose.pose.position.x=a-962557.903566
    gpose.pose.pose.position.y=b-1959165.24075
    gpose.pose.covariance[0]=cov1
    gpose.pose.covariance[7]=cov2
    gpose.pose.covariance[14]=cov3
    gpose.pose.covariance[21]=0
    gpose.pose.covariance[28]=0
    gpose.pose.covariance[35]=0

    roll,pitch,yaw = euler_from_quaternion(imu.quaternion.x,imu.quaternion.y,imu.quaternion.z,imu.quaternion.w)

    yaw_error = 0.27
    yaw = yaw + yaw_error

    qx,qy,qz,qw=get_quaternion_from_euler(roll,pitch,yaw)

    gpose.pose.pose.orientation.x = qx
    gpose.pose.pose.orientation.y = qy
    gpose.pose.pose.orientation.z = qz
    gpose.pose.pose.orientation.w = qw

    Odom_Pub.publish(gpose)


if __name__=='__main__':
	
    rospy.init_node('gps_imu_decision')
    gpose=Odometry()
    gpose.header.stamp=rospy.Time.now()
    gpose.header.frame_id="velodyne"

    gps_sub = message_filters.Subscriber("/gps/fix",NavSatFix)
    imu_sub = message_filters.Subscriber("/filter/quaternion",QuaternionStamped)

    Odom_Pub = rospy.Publisher('/odom', Odometry, queue_size=1)
    ts = message_filters.ApproximateTimeSynchronizer([gps_sub,imu_sub], 1,10)
    ts.registerCallback(callback)
    rospy.spin()

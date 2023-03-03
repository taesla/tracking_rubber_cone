#!/usr/bin/env python

import rospy
import struct
import math
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
import sensor_msgs.point_cloud2 as pc2
import numpy as np

cone_x=[]
cone_y=[]
cone_z=[]
cone_color=[]
absolute_points_list =[]
odom_x =0
odom_y =0
odom_yaw =0

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

def callback2(msg):
    global odom_x,odom_y,odom_yaw

    odom_x = msg.pose.pose.position.x
    odom_y = msg.pose.pose.position.y
    odom_yaw = euler_from_quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)

cnt = 0
def callback(msg):
   
    for p in pc2.read_points(msg , field_names = ("x","y","z","rgb"), skip_nans = True):
        
        if len(absolute_points_list) == 0 :
            absolute_points = (p[0]+odom_x , p[1]+odom_y , p[2],p[3],0.0)
            absolute_points_list.append(absolute_points)
            cnt = len(absolute_points_list)
        elif len(absolute_points_list) == len(p):
            cnt = len(absolute_points_list)
        else:
            cnt = len(p)*len(absolute_points_list)

        for k in range(cnt):
            if math.sqrt((absolute_points_list[k][0]-absolute_points[0])**2 + (absolute_points_list[k][1]-absolute_points[1])**2) < 1 :
                absolute_points_list[k] = [p[0]+odom_x,p[1]+odom_y]
            else:
                absolute_points_list + [p[0]+odom_x,p[1]+odom_y]
    
    for l in range(len(absolute_points_list)):
        if (absolute_points_list[l][0]-odom_x) < -0.3:
            del absolute_points_list[l]

    fields = [PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.UINT32, 1),
            PointField('intensity', 16, PointField.FLOAT32, 1)]
    print(len(absolute_points_list))
    sorted_points.header = msg.header   
    sorted_points.header.stamp = rospy.Time.now() 
    sorted_points.header.frame_id = 'map'
    sorted_pub.publish(pc2.create_cloud(sorted_points.header,fields,absolute_points_list))




if __name__=='__main__':
    rospy.init_node('mapping')

    rospy.Subscriber("/sorted_points1",PointCloud2,callback)
    rospy.Subscriber("/odom",Odometry,callback2)

    sorted_points = PointCloud2()

    sorted_pub = rospy.Publisher("/sorted_points2",PointCloud2,queue_size=1)

    rospy.spin()
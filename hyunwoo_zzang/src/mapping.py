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

odom_x =0
odom_y =0
odom_yaw =0

i=0

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


def callback(msg):
    global absolute_points_list, i
    tmp_list = []
    flag =True
    for p in pc2.read_points(msg , field_names = ("x","y","z","rgb"), skip_nans = True):
        absolute_points = [p[0]*np.cos(odom_yaw)-p[1]*np.sin(odom_yaw)+odom_x , p[0]*np.sin(odom_yaw)+p[1]*np.cos(odom_yaw)+odom_y , p[2],p[3],1]
        if i == 0 :
            absolute_points_list.append(absolute_points)
        else:    
            for k in range(len(absolute_points_list)):
                
                if math.sqrt(((absolute_points_list[k][0] - absolute_points[0])**2) + ((absolute_points_list[k][1] - absolute_points[1])**2)) < 0.3 :  
                    absolute_points_list[k] = absolute_points
                    flag = False
                    break
            if flag == False:
                continue
            tmp_list.append(absolute_points)

    i=1

                
    absolute_points_list =  absolute_points_list+tmp_list
    length = len(absolute_points_list)

    for l in range(length):
        if (np.cos(odom_yaw) * (absolute_points_list[l][0]-odom_x) + np.sin(odom_yaw) * (absolute_points_list[l][1]-odom_y)) < -1:
            del absolute_points_list[l]

    fields = [PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.UINT32, 1),
            PointField('intensity', 16, PointField.FLOAT32, 1)]
    print(len(absolute_points_list))
    sorted_points.header = msg.header   
    sorted_points.header.stamp = rospy.Time.now() 
    sorted_points.header.frame_id = 'velodyne'
    sorted_pub.publish(pc2.create_cloud(sorted_points.header,fields,absolute_points_list))




if __name__=='__main__':
    rospy.init_node('mapping')

    rospy.Subscriber("/sorted_points1",PointCloud2,callback)
    rospy.Subscriber("/odom",Odometry,callback2)

    sorted_points = PointCloud2()
    absolute_points_list =[]
    sorted_pub = rospy.Publisher("/sorted_points2",PointCloud2,queue_size=1)

    rospy.spin()

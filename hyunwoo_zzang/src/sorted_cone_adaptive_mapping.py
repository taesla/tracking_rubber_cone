#!/usr/bin/env python

import rospy
import struct
import math
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
import sensor_msgs.point_cloud2 as pc2

import numpy as np
absolute_points_list = []
minimum_points =[]
yaw= 0

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


def callback3(msg):
    global odom, yaw

    
    odom = msg
    yaw = euler_from_quaternion(odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w)

def callback2(msg):

    global cone_centers
    cone_centers = PoseArray()
    cone_centers = msg

def callback(msg):

    if msg.header.frame_id[0]=='b':
        cone_number = msg.header.frame_id[4]
    if msg.header.frame_id[0]=='y':
        cone_number = msg.header.frame_id[6]
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.UINT32, 1),
            PointField('intensity', 16, PointField.FLOAT32, 1)]
    if cone_number == '0':
        print('publish')
        if not len(absolute_points_list) == 0:
            sorted_point_pub.publish(pc2.create_cloud(sorted_points.header,fields,absolute_points_list))


    cone_x = []
    cone_y = []
    cone_z = []

    for p in pc2.read_points(msg , field_names = ("x","y","z"), skip_nans = True):
        cone_x.append(p[0])
        cone_y.append(p[1])
        cone_z.append(p[2])
    if (len(cone_centers.poses)) > 0 :
        for i in range(len(cone_centers.poses)):
            for j in range(len(cone_x)):
                if math.sqrt((cone_x[j]-cone_centers.poses[i].position.x)**2 + (cone_y[j]-cone_centers.poses[i].position.y)**2) < 0.2 :
                    pt = [cone_centers.poses[i].position.x,cone_centers.poses[i].position.y,0,0,0]
                    if msg.header.frame_id[0]=='b':
                        r = int(0 * 255.0)
                        g = int(0 * 255.0)
                        b = int(1 * 255.0)
                        a = 255
                    else:
                        r = int(1 * 255.0)
                        g = int(1 * 255.0)
                        b = int(0 * 255.0)
                        a = 255
                    rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                    pt[3] = rgb
                    pt[4] = 1

                    absolute_points = [pt[0]+odom.pose.pose.position.x * np.cos(yaw + np.pi / 2 ), pt[1]+odom.pose.pose.position.y * np.sin(yaw + np.pi / 2 ), pt[2],pt[3],pt[4]]
                    
                    if len(absolute_points_list) == 0 :
                        absolute_points_list.append(absolute_points)
                    for k in range(len(absolute_points_list)):
                        if math.sqrt((absolute_points_list[k][0]-absolute_points[0])**2 + (absolute_points_list[k][1]-absolute_points[1])**2) < 0.3 :
                            absolute_points_list[k] = absolute_points
                        else:
                            absolute_points_list.append(absolute_points)
                        if (absolute_points_list[k][0]-odom.pose.pose.position.x * np.cos(yaw + np.pi / 2)) < 3.3:
                            print(absolute_points_list[k][0],absolute_points_list[k][1])
                            del absolute_points_list[k]

                    break

    # for l in range(len(absolute_points_list)):
    #     minimum_points.append(absolute_points_list[l][0]-odom.pose.pose.position.x * np.cos(yaw + np.pi / 2), absolute_points_list[l][1] - odom.pose.pose.position.y * np.sin(yaw + np.pi / 2),absolute_points_list[l][2],absolute_points_list[l][3],absolute_points_list[l][4])

    # print(msg.header.frame_id)
    sorted_points.header = msg.header   
    sorted_points.header.stamp = rospy.Time.now() 
    sorted_points.header.frame_id = 'velodyne'


if __name__=='__main__':

    rospy.init_node('cone_distance')
    rospy.Subscriber("/id_points2",PointCloud2,callback)
    rospy.Subscriber("/adaptive_clustering/poses",PoseArray,callback2)
    rospy.Subscriber("/odom",Odometry,callback3)
    sorted_point_pub = rospy.Publisher("/sorted_points2",PointCloud2,queue_size=1)
    
    cone_centers = PoseArray()
    odom = Odometry()
    sorted_points= PointCloud2()

    rospy.spin()

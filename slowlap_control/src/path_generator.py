#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point, PoseStamped, PoseArray
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA, Float32
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Path, Odometry

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

def callback2(msg):
    global tmp_pose
    tmp_pose = PoseArray()
    for i in range(len(msg.poses)):
        if ((msg.poses[i].position.x>-0.8) and (msg.poses[i].position.x<0.8) and (msg.poses[i].position.y>-0.8) and (msg.poses[i].position.y<0.8)):
            continue
        else:
            tmp_pose.poses.append(msg.poses[i])
            continue

def callback(msg):
    
    blue_x = []
    blue_y = []

    yellow_x = []
    yellow_y = []

    mid_path_points_x = []
    mid_path_points_y = []

    for p in pc2.read_points(msg , field_names = ("x","y","z","rgb"), skip_nans = True):

        if p[3] == 4278190335 :
            blue_x.append(p[0])
            blue_y.append(p[1])

        else :
            yellow_x.append(p[0])
            yellow_y.append(p[1])

    if (len(blue_x) ==0 and not len(yellow_x)==0) ==1: # only yellow
        blue_candid_x =[]
        blue_candid_y =[]
        for i in range(len(tmp_pose.poses)):
            if tmp_pose.poses[i].position.x>0 and tmp_pose.poses[i].position.y<0:
                blue_candid_x.append(tmp_pose.poses[i].position.x)
                blue_candid_y.append(tmp_pose.poses[i].position.y)
        ind = blue_candid_x.index(min(blue_candid_x))
        blue_x.append(blue_candid_x[ind])
        blue_y.append(blue_candid_y[ind])

    if (not len(blue_x) ==0 and len(yellow_x)==0) ==1: # only blue
        yellow_candid_x =[]
        yellow_candid_y =[]
        for i in range(len(tmp_pose.poses)):
            if tmp_pose.poses[i].position.x>0 and tmp_pose.poses[i].position.y>0:
                yellow_candid_x.append(tmp_pose.poses[i].position.x)
                yellow_candid_y.append(tmp_pose.poses[i].position.y)
        ind = yellow_candid_x.index(min(yellow_candid_x))
        yellow_x.append(yellow_candid_x[ind])
        yellow_y.append(yellow_candid_y[ind])

    for i in range(len(blue_x)):
        for j in range(len(yellow_x)):

            mid_path_points_x.append(( blue_x[i] + yellow_x[j] ) / 2)
            mid_path_points_y.append(( blue_y[i] + yellow_y[j] ) / 2)
            # sort_list(mid_path_points_y,mid_path_points_x)
            # mid_path_points_x.sort()

    x=np.linspace(0,5,100)
    # mid_path_points_x.insert(0,4)
    # mid_path_points_y.insert(0,0)
    # mid_path_points_x.insert(0,3)
    # mid_path_points_y.insert(0,0)
    # mid_path_points_x.insert(0,2)
    # mid_path_points_y.insert(0,0)
    #mid_path_points_x.insert(0,1)
    #mid_path_points_y.insert(0,0)
    mid_path_points_x.insert(0,-0.2)
    mid_path_points_y.insert(0,0)
    mid_path_points_x.insert(0,-0.4)
    mid_path_points_y.insert(0,0)
    mid_path_points_x.insert(0,-0.6)
    mid_path_points_y.insert(0,0)
    mid_path_points_x.insert(0,-0.8)
    mid_path_points_y.insert(0,0)
    mid_path_points_x.insert(0,-1.0)
    mid_path_points_y.insert(0,0)
    weights = []

    for i in range(len(mid_path_points_x)):
        if i < 3:
            weights.append(22)
        else :
            weights.append(1)
    p = np.polyfit(mid_path_points_x,mid_path_points_y,2,w = weights)
    path_func = p[0] * x ** 2 + p[1] * x + p[2]
    # path_func = p[0] * x + p[1]

    mid_points = Marker()
    mid_points.header.frame_id = 'velodyne'
    mid_points.type = Marker.POINTS
    mid_points.action = Marker.ADD

    del mid_points.points[:]
    for i in range(len(mid_path_points_x)):
        mid_points.points.append(Point(mid_path_points_x[i],mid_path_points_y[i],0))
    mid_points.color = ColorRGBA(1, 1, 1, 1)
    mid_points.scale.x = 0.1
    mid_points.scale.y = 0.1
    mid_points.scale.z = 0
   

 
    path = Path()
    path.header.frame_id = 'velodyne'
    
    for i in range(len(x)-1):
        pose = PoseStamped()
        pose.pose.position.x = x[i]
        pose.pose.position.y = path_func[i]
        pose.pose.position.z = 0

        yaw = np.arctan2(path_func[i+1]-path_func[i],x[i+1]-x[i] )
        qx_,qy_,qz_,qw_ = get_quaternion_from_euler(0,0,yaw)
        pose.pose.orientation.x = qx_
        pose.pose.orientation.y = qy_
        pose.pose.orientation.z = qz_
        pose.pose.orientation.w = qw_
        path.poses.append(pose)

    path_pub.publish(path)

    mid_pub.publish(mid_points)




if __name__=='__main__':
    rospy.init_node('slowlap_control')
    
    rospy.Subscriber("sorted_points2",PointCloud2,callback)
    rospy.Subscriber("/adaptive_clustering/poses",PoseArray,callback2)

    path_pub = rospy.Publisher("/waypoints",Path,queue_size=1)
    mid_pub = rospy.Publisher("cone_mid_points",Marker,queue_size=1)

    rospy.spin()

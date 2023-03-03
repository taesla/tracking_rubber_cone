#!/usr/bin/env python

from glob import glob
import rospy
import struct
import math
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseArray
import sensor_msgs.point_cloud2 as pc2

test_list = []

def callback2(msg):
    j=1

def callback(msg):
    i=0
    for p in pc2.read_points(msg , field_names = ("x","y","z"), skip_nans = True):
        test_list.append(i)
        i=i+1

def callback3(msg):
    i=0
    for p in pc2.read_points(msg , field_names = ("x","y","z"), skip_nans = True):
        test_list.append(i)
        i=i+1

    print(test_list)







if __name__=='__main__':

    rospy.init_node('cone_distance')
    rospy.Subscriber("/id_points2",PointCloud2,callback)
    rospy.Subscriber("/id_points2_blue",PointCloud2,callback3)
    rospy.Subscriber("/adaptive_clustering/poses",PoseArray,callback2)
    sorted_point_pub = rospy.Publisher("/sorted_points2",PointCloud2,queue_size=1)
    
    cone_centers = PoseArray()
    sorted_points= PointCloud2()

    rospy.spin()